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
    let nrel_api_key =
        std::env::var("NREL_API_KEY").expect("NREL_API_KEY environment variable is not set");
    let charger_locations = download_source_data(&nrel_api_key).unwrap();
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
    let client = Client::new();
    let distance = ny.get_osrm_distance(DEFAULT_OSRM_URL, &client, &test_atlanta_charger);
    println!("distance: {:?}", distance);
}

const US_BOUNDING_BOX: BoundingBox = BoundingBox {
    lat_min: 24.5243,
    lon_min: -124.7580,
    lat_max: 49.1756,
    lon_max: -66.9472,
};

// our numbers are kind of big, so we don't super care about floating point error accumulation in the last couple of decimal places.
const REASONABLE_EPSILON: f64 = 0.00000000000001;
macro_rules! assert_float_eq {
    ($a:expr, $b:expr) => {
        assert!(
            $a - $b < REASONABLE_EPSILON,
            "floats equal: {} == {}",
            $a,
            $b
        )
    };
}

#[test]
fn chunkify_correct_number_of_chunks() {
    for chunks in [4, 6, 8, 10, 12] {
        assert_eq!(US_BOUNDING_BOX.chunkify(chunks).len(), chunks)
    }
}

#[test]
fn chunkify_correct_width() {
    for n_chunks in [4, 6, 8, 10, 12] {
        let width = US_BOUNDING_BOX.width() / (n_chunks / 2) as f64;
        let chunks = US_BOUNDING_BOX.chunkify(n_chunks);
        for chunk in chunks {
            assert_float_eq!(chunk.width(), width);
        }
    }
}

#[test]
fn chunkify_correct_height() {
    for n_chunks in [4, 6, 8, 10, 12] {
        let height = US_BOUNDING_BOX.height();
        let chunks = US_BOUNDING_BOX.chunkify(n_chunks);
        for chunk in chunks {
            assert_float_eq!(chunk.height(), height);
        }
    }
}

#[test]
fn chunks_end_in_correct_places() {
    for n_chunks in [4, 6, 8, 10, 12] {
        let chunks = US_BOUNDING_BOX.chunkify(n_chunks);

        let first = chunks.first().unwrap();
        assert_float_eq!(first.lat_min, US_BOUNDING_BOX.lat_min);
        assert_float_eq!(first.lon_min, US_BOUNDING_BOX.lon_min);
        assert_float_eq!(first.lon_max, US_BOUNDING_BOX.lon_max);

        let last = chunks.last().unwrap();
        assert_float_eq!(last.lat_max, US_BOUNDING_BOX.lat_max);
        assert_float_eq!(last.lon_min, US_BOUNDING_BOX.lon_min);
        assert_float_eq!(last.lon_max, US_BOUNDING_BOX.lon_max);
    }
}
