use core::f64;
use std::error::Error;

use serde::Deserialize;

/// Assumed EV's max range in meters.
pub const MAX_RANGE_METERS: u64 = 400_000;
/// Ratio between the linear distance and driving distance at which we assume
/// point is reachable, intended to avoid API lookups for chargers that are
/// obviously reachable.
pub const CROW_FLIES_RATIO: f64 = 0.1;

#[derive(Clone, Debug, Deserialize)]
pub struct ChargerLocation {
    #[serde(rename = "Latitude")]
    latitude: f64,
    #[serde(rename = "Longitude")]
    longitude: f64,
    #[serde(rename = "ID")]
    id: u64,
}

pub struct TrialPoint {
    pub latitude: f64,
    pub longitude: f64,
}

pub fn read_csv(path_to_csv: &str) -> Result<Vec<ChargerLocation>, Box<dyn Error>> {
    // Build the CSV reader and iterate over each record.
    let mut rdr = csv::Reader::from_path(path_to_csv)?;
    let mut chargers = Vec::new();
    for result in rdr.deserialize() {
        // The iterator yields Result<StringRecord, Error>, so we check the
        // error here.
        let record: ChargerLocation = result?;
        chargers.push(record);
    }
    Ok(chargers)
}

pub enum CheckResult {
    Yes,
    No,
    Maybe {
        candidates: Vec<(ChargerLocation, u64)>,
    },
}

impl TrialPoint {
    pub fn check_charger(self, chargers: &[ChargerLocation]) -> CheckResult {
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
        use std::f64::consts::PI;
        const EARTH_RADIUS_METERS: f64 = 6_371_000.0;

        let lat1 = self.latitude * (PI / 180.);
        let lat2 = charger.latitude * (PI / 180.);

        let delta_lat = (self.latitude - charger.latitude) * (PI / 180.);
        let delta_lon = (self.longitude - charger.longitude) * (PI / 180.);
        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1.cos() * (lat2.cos()) * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        EARTH_RADIUS_METERS * c
    }

    pub fn nearest_chargers(&self, chargers: &[ChargerLocation]) -> Vec<(ChargerLocation, u64)> {
        let mut charger_distances = Vec::new();
        for c in chargers {
            let dist = self.distance_to(c) as u64;
            if dist < MAX_RANGE_METERS {
                charger_distances.push((c.clone(), dist));
            }
        }
        charger_distances.sort_by_key(|(_, distance)| *distance);
        charger_distances
    }
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
        };
        let distance = ny.distance_to(&la);
        let error = (3950_000. - distance).abs();
        // Assert that we're within 50km as a sanity check
        assert!(error < 50_000.);
    }
}
