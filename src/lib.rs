use core::f64;
use std::{collections::HashMap, error::Error, thread, time::Duration};

use csv::Reader;
use geo::MultiPoint;
use geo::{algorithm::concave_hull::ConcaveHull, Polygon};
use quadtree_f32::{Item, ItemId, Point, QuadTree, Rect};
use reqwest;
use reqwest::blocking::Client;
use serde::Deserialize;
use std::f64::consts::PI;

/// Assumed EV's max range in meters.
pub const MAX_RANGE_METERS: u64 = 400_000;
/// Ratio between the linear distance and driving distance at which we assume
/// point is reachable, intended to avoid API lookups for chargers that are
/// obviously reachable.
pub const CROW_FLIES_RATIO: f64 = 0.1;
pub const EARTH_RADIUS_METERS: f64 = 6_371_000.0;
pub const DEFAULT_OSRM_URL: &'static str = "https://router.project-osrm.org";

#[cfg(test)]
mod tests;

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

#[derive(Clone)]
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
#[derive(Debug)]
pub struct BoundingBox {
    pub lat_min: f64,
    pub lat_max: f64,
    pub lon_min: f64,
    pub lon_max: f64,
}

impl AllChargerLocations {
    pub fn find_gaps(
        &self,
        resolution: f64,
        bbox: BoundingBox,
        osrm_url: &str,
        client: Client,
    ) -> geo::Polygon<f64> {
        let grid = bbox.generate_grid(resolution);
        let total = grid.len();
        let thread = thread::current().id();
        println!("{:?} generated grid (length: {})", thread, total);
        let mut reachable = 0;
        let mut unreachable = 0;
        let mut maybe_reachable = 0;
        let mut api_call_counter = 0;
        let start = std::time::Instant::now();
        let mut not_reachable_points = Vec::new();
        for (i, point) in grid.into_iter().enumerate() {
            let result = point.check_charger(&self);
            match result {
                CheckResult::Yes => {
                    reachable += 1;
                }
                CheckResult::No => {
                    unreachable += 1;
                    let geo_point = geo::Point::new(point.latitude, point.longitude);
                    not_reachable_points.push(geo_point);
                }
                CheckResult::Maybe { candidates } => {
                    maybe_reachable += 1;
                    // Find the distance between points and chargers that are maybe reachable
                    // Where candidates is a vector of ChargerLocations
                    let mut is_reachable = false;
                    let mut tried_chargers = 0;
                    for (charger, _) in candidates {
                        if let Some(distance) = point.get_osrm_distance(osrm_url, &client, &charger)
                        {
                            api_call_counter += 1;
                            if distance as u64 <= MAX_RANGE_METERS {
                                reachable += 1;
                                is_reachable = true;
                                break;
                            }
                            tried_chargers += 1;
                            if tried_chargers == 50 {
                                break;
                            }
                        }
                    }
                    if is_reachable == false {
                        unreachable += 1;
                        let geo_point = geo::Point::new(point.longitude, point.latitude);
                        not_reachable_points.push(geo_point);
                    }
                }
            }
            if i % 1_000 == 0 {
                println!("{:?} {}: {:?}", thread, i, start.elapsed());
                println!("{:?} reachable: {}", thread, reachable);
                println!("{:?} unreachable: {}", thread, unreachable);
                println!("{:?} maybe reachable: {}", thread, maybe_reachable);
                println!("{:?} api call count: {}", thread, api_call_counter);
                api_call_counter = 0;
            }
        }
        println!(
            "{:?} DONE Resolution: {}\n\nTotal points: {}\nReachable: {}\nUnreachable: {}\nUnknown: {}",
            thread, resolution, total, reachable, unreachable, maybe_reachable
        );
        let multipoint = MultiPoint(not_reachable_points);
        println!("{:?} Before concave_hull: {:?}", thread, start.elapsed());
        multipoint.concave_hull(2.0) // Documentation uses 2 as example concavity
    }
}

pub fn download_source_data(nrel_api_key: &str) -> Result<AllChargerLocations, Box<dyn Error>> {
    let url = format!("https://developer.nrel.gov/api/alt-fuel-stations/v1.csv?access=public&api_key={}&cards_accepted=all&cng_fill_type=all&cng_psi=all&cng_vehicle_class=all&country=all&download=true&e85_has_blender_pump=false&ev_charging_level=2%2Cdc_fast&ev_connector_type=all&ev_network=all&fuel_type=ELEC&hy_is_retail=true&limit=all&lng_vehicle_class=all&lpg_include_secondary=false&offset=0&owner_type=all&state=all&status=E&utf8_bom=true", nrel_api_key);
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
    pub fn check_charger(&self, chargers: &AllChargerLocations) -> CheckResult {
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

    pub fn get_osrm_distance(
        &self,
        osrm_url: &str,
        client: &Client,
        charger: &ChargerLocation,
    ) -> Option<f64> {
        let osrm_api_url = format!(
            "{}/route/v1/driving/{},{};{},{}",
            osrm_url, self.longitude, self.latitude, charger.longitude, charger.latitude
        );
        let mut retries = 0;
        let body = loop {
            match client
                .get(osrm_api_url.clone())
                .send()
                .and_then(|rsp| rsp.text())
            {
                Ok(body) => match serde_json::from_str::<Json>(&body) {
                    Ok(json) => break json,
                    // If we get a response back (the request succeeded) but the response doesn't have
                    // valid response json, we assume there is no possible path between those pts
                    Err(error) => {
                        println!(
                            "{:?} retrying ({}) body error: {}\nbody: {}",
                            thread::current().id(),
                            retries,
                            error,
                            body,
                        );
                        return None;
                    }
                },
                Err(error) => println!(
                    "{:?} retrying ({}) request error: {}",
                    thread::current().id(),
                    retries,
                    error
                ),
            };
            retries += 1;
            let sleep = if retries > 60 { 60 } else { retries };
            thread::sleep(Duration::from_secs(sleep));
        };
        let distance = body.routes[0].distance;
        Some(distance)
    }
}

pub fn add_meters_to_coords(meters: f64, (lat, lon): (f64, f64)) -> (f64, f64) {
    let degrees_lat = lat + (meters / EARTH_RADIUS_METERS) * (180.0 / PI);
    let degrees_lon =
        lon + (meters / EARTH_RADIUS_METERS) * (180.0 / PI) / (lat * PI / 180.0).cos();
    (degrees_lat, degrees_lon)
}

impl BoundingBox {
    pub fn generate_grid(self, resolution: f64) -> Vec<TrialPoint> {
        let number_lat_pts = ((self.width()) / resolution) as u64;
        let number_lon_pts = ((self.height()) / resolution) as u64;
        println!(
            "{:?} generating {} x {} grid",
            thread::current().id(),
            number_lat_pts,
            number_lon_pts
        );
        let mut grid = Vec::with_capacity((number_lat_pts * number_lon_pts) as usize);
        for lat in 0..number_lat_pts {
            for lon in 0..number_lon_pts {
                let latitude = self.lat_min + (lat as f64 * resolution);
                let longitude = self.lon_min + (lon as f64 * resolution);
                grid.push(TrialPoint {
                    latitude,
                    longitude,
                });
            }
        }
        grid
    }
    pub fn width(&self) -> f64 {
        // By taking the absolute value, this works in both hemispheres
        (self.lat_min - self.lat_max).abs()
    }
    pub fn height(&self) -> f64 {
        (self.lon_min - self.lon_max).abs()
    }
    pub fn chunkify(self, chunks: usize) -> Vec<BoundingBox> {
        let mut chunks_vec = Vec::new();
        let width_interval = self.width() / ((chunks / 2) as f64);
        for num_width_intervals in 0..chunks {
            let current_width_interval = num_width_intervals as f64 * width_interval;
            let chunk = BoundingBox {
                lat_min: self.lat_min - current_width_interval,
                lat_max: self.lat_max - (current_width_interval + width_interval),
                lon_min: self.lon_min,
                lon_max: self.lon_max,
            };
            chunks_vec.push(chunk);
        }
        assert_eq!(chunks_vec.len(), chunks);
        chunks_vec
    }
}
