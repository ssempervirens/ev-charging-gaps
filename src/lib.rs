use core::f64;
use std::{collections::HashMap, error::Error};

use csv::Reader;
use quadtree_f32::{Item, ItemId, Point, QuadTree, Rect};
use reqwest;
use serde::Deserialize;
use std::f64::consts::PI;

/// Assumed EV's max range in meters.
pub const MAX_RANGE_METERS: u64 = 400_000;
/// Ratio between the linear distance and driving distance at which we assume
/// point is reachable, intended to avoid API lookups for chargers that are
/// obviously reachable.
pub const CROW_FLIES_RATIO: f64 = 0.1;
pub const EARTH_RADIUS_METERS: f64 = 6_371_000.0;

/// CsvRow includes all information we need about chargers
/// that is parsed out from CSV row
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct CsvRow {
    #[serde(rename = "Latitude")]
    latitude: f64,
    #[serde(rename = "Longitude")]
    longitude: f64,
    #[serde(rename = "ID")]
    id: u64,
    #[serde(rename = "EV Network")]
    network: String,
}

/// All operations done on ChargerLocations type
#[derive(Clone, Debug, PartialEq)]
pub struct ChargerLocation {
    latitude: f64,
    longitude: f64,
    id: u64,
}

pub struct TrialPoint {
    pub latitude: f64,
    pub longitude: f64,
}

pub struct AllChargerLocations {
    pub quadtree: QuadTree,
    pub chargers_by_id: HashMap<ItemId, ChargerLocation>,
}

#[derive(Deserialize, Debug)]
pub struct Json {
    pub routes: Vec<Route>,
}

#[derive(Deserialize, Debug)]
pub struct Route {
    pub distance: f64,
}

pub fn download_source_data() -> Result<AllChargerLocations, Box<dyn Error>> {
    let url = "https://developer.nrel.gov/api/alt-fuel-stations/v1.csv?access=public&api_key=oMa5C8ffgw2DXGNv7HHaWSZKWx2rGeBGdkfLvL70&cards_accepted=all&cng_fill_type=all&cng_psi=all&cng_vehicle_class=all&country=all&download=true&e85_has_blender_pump=false&ev_charging_level=2%2Cdc_fast&ev_connector_type=all&ev_network=all&fuel_type=ELEC&hy_is_retail=true&limit=all&lng_vehicle_class=all&lpg_include_secondary=false&offset=0&owner_type=all&state=all&status=E&utf8_bom=true";
    let body = reqwest::blocking::get(url)?.text()?;
    let reader = Reader::from_reader(body.as_bytes());
    read_csv(reader)
}

pub fn read_from_file(path_to_csv: &str) -> Result<AllChargerLocations, Box<dyn Error>> {
    let reader = csv::Reader::from_path(path_to_csv)?;
    read_csv(reader)
}

pub fn read_csv<R>(mut reader: csv::Reader<R>) -> Result<AllChargerLocations, Box<dyn Error>>
where
    R: std::io::Read,
{
    let mut chargers_by_id = HashMap::new();
    let rows = reader
        .deserialize()
        .filter_map(|row: Result<CsvRow, _>| row.ok())
        // We are interested in the gaps in non-Tesla charging infrastructure
        // TODO: might be interesting to make that a command line argument so
        // we can see gaps in other networks
        .filter(|row| !row.network.contains("Tesla"))
        .map(|location| {
            let id = ItemId(location.id as usize);
            let point = Item::Point(Point {
                x: location.latitude as f32,
                y: location.longitude as f32,
            });
            // Because we don't need to copy the network strings all the time, just use ChargerLocation type
            // so we convert csv row into ChargerLocation
            chargers_by_id.insert(
                id,
                ChargerLocation {
                    latitude: location.latitude,
                    longitude: location.longitude,
                    id: location.id,
                },
            );
            (id, point)
        });
    let quadtree = QuadTree::new(rows);
    println!(
        "tree = {:?}; len = {}",
        quadtree.bbox(),
        chargers_by_id.len()
    );
    Ok(AllChargerLocations {
        quadtree,
        chargers_by_id,
    })
}

pub enum CheckResult {
    Yes,
    No,
    Maybe {
        candidates: Vec<(ChargerLocation, u64)>,
    },
}

impl TrialPoint {
    pub fn check_charger(self, chargers: &AllChargerLocations) -> CheckResult {
        let nearest_chargers = self.nearest_chargers(chargers);

        // If there are no chargers within MAX_RANGE_METERS, the list will be empty;
        // this point cannot be reachable based on driving distance if all crow-flies
        // distances are greater.
        if nearest_chargers.is_empty() {
            return CheckResult::No;
        }

        // Because the list of chargers is sorted, the first element is
        // always the closest, so we can use it for checking if the point
        // is trivially reachable.
        let nearest_charger_distance = nearest_chargers[0].1;

        // If the nearest charger is really close, this point *definitely* has a
        // reachable charger.
        if nearest_charger_distance < (MAX_RANGE_METERS as f64 * CROW_FLIES_RATIO) as u64 {
            CheckResult::Yes
        } else {
            // We need to use the OSRM API to find out whether a charger is reachable.
            CheckResult::Maybe {
                candidates: nearest_chargers,
            }
        }
    }

    /// Returns the distance in kilometers from this `TrialPoint` to the given `charger`.
    pub fn distance_to(&self, charger: &ChargerLocation) -> f64 {
        // Calculate the distance using the Haversine formula

        let lat1 = self.latitude * (PI / 180.);
        let lat2 = charger.latitude * (PI / 180.);

        let delta_lat = (self.latitude - charger.latitude) * (PI / 180.);
        let delta_lon = (self.longitude - charger.longitude) * (PI / 180.);
        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1.cos() * (lat2.cos()) * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        EARTH_RADIUS_METERS * c
    }

    pub fn nearest_chargers(&self, chargers: &AllChargerLocations) -> Vec<(ChargerLocation, u64)> {
        // QuadTree uses 2 dimensional geometry, so we add padding to the bounding box to
        // ensure we get all possible relevant points since this is an approximation
        const PADDED_MAX_RANGE_METERS: f64 = MAX_RANGE_METERS as f64 + 25_000.0;
        let (max_x, max_y) =
            add_meters_to_coords(PADDED_MAX_RANGE_METERS, (self.latitude, self.longitude));
        let (min_x, min_y) =
            add_meters_to_coords(-PADDED_MAX_RANGE_METERS, (self.latitude, self.longitude));
        let bbox = Rect {
            max_x: max_x as f32,
            max_y: max_y as f32,
            min_x: min_x as f32,
            min_y: min_y as f32,
        };
        // println!(
        //     "bounding box {:?} at {:?} has dimensions {} x {}",
        //     bbox,
        //     bbox.get_center(),
        //     bbox.get_width(),
        //     bbox.get_height()
        // );
        let ids = chargers.quadtree.get_ids_that_overlap(&bbox);
        let mut chargers_distances = Vec::with_capacity(ids.len());
        for id in ids {
            if let Some(charger) = chargers.chargers_by_id.get(&id) {
                let distance = self.distance_to(&charger) as u64;
                chargers_distances.push((charger.clone(), distance));
            }
        }
        chargers_distances.sort_by_key(|(_, distance)| *distance);
        chargers_distances
    }

    pub fn get_osrm_distance(&self, charger: &ChargerLocation) -> f64 {
        let osrm_api_url = format!(
            "https://router.project-osrm.org/route/v1/driving/{},{};{},{}",
            self.longitude, self.latitude, charger.longitude, charger.latitude
        );
        let body = reqwest::blocking::get(osrm_api_url).unwrap().json::<Json>();
        let distance = body.unwrap().routes[0].distance;
        distance
    }
}

pub fn add_meters_to_coords(meters: f64, (lat, lon): (f64, f64)) -> (f64, f64) {
    let degrees_lat = lat + (meters / EARTH_RADIUS_METERS) * (180.0 / PI);
    let degrees_lon =
        lon + (meters / EARTH_RADIUS_METERS) * (180.0 / PI) / (lat * PI / 180.0).cos();
    (degrees_lat, degrees_lon)
}

pub fn generate_grid(
    resolution: f64,
    lat_start: f64,
    lon_start: f64,
    lat_end: f64,
    lon_end: f64,
) -> Vec<TrialPoint> {
    // ONLY WORKS IN NORTHERN HEMISPHERE LOL
    let number_lat_pts = ((lat_start - lat_end) / resolution) as u64;
    let number_lon_pts = ((lon_end - lon_start) / resolution) as u64;
    println!("generating {} x {} grid", number_lat_pts, number_lon_pts);
    let mut grid = Vec::with_capacity((number_lat_pts * number_lon_pts) as usize);
    for lat in 0..number_lat_pts {
        for lon in 0..number_lon_pts {
            let latitude = lat_start + (lat as f64 * resolution);
            let longitude = lon_start + (lon as f64 * resolution);
            grid.push(TrialPoint {
                latitude,
                longitude,
            });
        }
    }
    grid
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_ny_to_la() {
        let ny = TrialPoint {
            latitude: 40.730610,
            longitude: -73.935242,
        };
        let la = ChargerLocation {
            latitude: 34.052235,
            longitude: -118.243683,
            id: 1,
            // network: "Electrify America".to_string(),
        };
        let distance = ny.distance_to(&la);
        let error = (3950_000. - distance).abs();
        // Assert that we're within 50km as a sanity check
        assert!(error < 50_000.);
    }
    #[test]
    fn quadtree_include_relevant_points() {
        // Test that if we call our nearest_chargers function, test
        // that everything we get back is in our list of real chargers
        // If there is an error, we just want the test to fail
        let charger_locations = download_source_data().unwrap();
        let ny = TrialPoint {
            latitude: 40.730610,
            longitude: -73.935242,
        };
        let mut slow_check = Vec::new();
        for charger in charger_locations.chargers_by_id.values() {
            // Check if less than the max distance
            let distance = ny.distance_to(charger) as u64;
            if distance < MAX_RANGE_METERS {
                slow_check.push((charger.clone(), distance));
            }
        }
        slow_check.sort_by_key(|(_, distance)| *distance);
        let test_chargers = ny.nearest_chargers(&charger_locations);
        assert!(
            test_chargers.len() >= slow_check.len(),
            "nearest chargers must be at least as long as the expected nearest chargers"
        );
        println!(
            "{} check points;\n{} actual points",
            slow_check.len(),
            test_chargers.len()
        );
        for (i, expected) in slow_check.iter().enumerate() {
            assert_eq!(expected.1, (&test_chargers[i]).1);
        }
    }

    #[test]
    fn osrm_api_works() {
        let ny = TrialPoint {
            latitude: 40.730610,
            longitude: -73.935242,
        };
        let test_atlanta_charger = ChargerLocation {
            latitude: 33.75,
            longitude: -84.4,
            id: 666,
        };
        let distance = ny.get_osrm_distance(&test_atlanta_charger);
        assert_eq!(distance, 585776.3)
    }
}
