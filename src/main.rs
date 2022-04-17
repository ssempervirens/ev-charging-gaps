use clap::Parser;
use geo::algorithm::concave_hull::ConcaveHull;
use geo::MultiPoint;
use reqwest::blocking::Client;
use shapefile::dbase;
use shapefile::Polygon;
use std::error::Error;

use ev_charging_gaps::*;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Path to charger csv file
    ///
    /// If this is not provided, need API key to download
    /// charger location data
    #[clap(short, long, required_unless_present = "nrel-api-key")]
    path: Option<String>,
    /// Grid resolution, in degrees.
    #[clap(short, long, default_value_t = 0.01)]
    resolution: f64,
    /// Base url of OSRM server, default is public API
    #[clap(short, long, default_value = DEFAULT_OSRM_URL)]
    osrm_url: String,
    /// API key for the downloading NREL charger data
    ///
    /// Only needed if path is not set
    #[clap(long, env = "NREL_API_KEY", required_unless_present = "path")]
    nrel_api_key: Option<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    // Bounding box of continental United States
    let lat_start = 49.1756;
    let lon_start = -124.7580;
    let lat_end = 24.5243;
    let lon_end = -66.9472;

    let charger_locations = match args.path {
        Some(path) => read_from_file(&path),
        None => download_source_data(
            &args
                .nrel_api_key
                .expect("If there was no path provided, there should be a NREL API key"),
        ),
    }?;
    let grid = generate_grid(args.resolution, lat_start, lon_start, lat_end, lon_end);
    let total = grid.len();
    println!("generated grid (length: {})", total);
    let mut reachable = 0;
    let mut unreachable = 0;
    let mut maybe_reachable = 0;
    let mut api_call_counter = 0;
    let client = Client::new();
    let start = std::time::Instant::now();
    let mut not_reachable_points = Vec::new();
    for (i, point) in grid.into_iter().enumerate() {
        let result = point.check_charger(&charger_locations);
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
                for (charger, _) in candidates {
                    let distance =
                        point.get_osrm_distance(&args.osrm_url, &client, &charger) as u64;
                    api_call_counter += 1;
                    if distance <= MAX_RANGE_METERS {
                        reachable += 1;
                        is_reachable = true;
                        break;
                    }
                }
                if is_reachable == false {
                    unreachable += 1;
                    let geo_point = geo::Point::new(point.latitude, point.longitude);
                    not_reachable_points.push(geo_point);
                }
            }
        }
        if i % 1_000 == 0 {
            println!("{}: {:?}", i, start.elapsed());
            println!("reachable: {}", reachable);
            println!("unreachable: {}", unreachable);
            println!("maybe reachable: {}", maybe_reachable);
            println!("api call count: {}", api_call_counter);
            api_call_counter = 0;
        }
    }
    println!(
        "Resolution: {}\n\nTotal points: {}\nReachable: {}\nUnreachable: {}\nUnknown: {}",
        args.resolution, total, reachable, unreachable, maybe_reachable
    );
    let multipoint = MultiPoint(not_reachable_points);
    println!("Before concave_hull: {:?}", start.elapsed());
    let concave_hull = multipoint.concave_hull(2.0); // Documentation uses 2 as example concavity
    println!("After concave_hull: {:?}", start.elapsed());
    let table_info = dbase::TableWriterBuilder::new()
        .add_logical_field(dbase::FieldName::try_from("has_charger").unwrap());
    let mut writer = shapefile::Writer::from_path("output/test_shapefile.shp", table_info)?;
    let mut record = dbase::Record::default();
    record.insert(
        "has_charger".to_owned(),
        dbase::FieldValue::Logical(Some(false)),
    );
    let converted_polygon = Polygon::from(concave_hull);
    writer.write_shape_and_record(&converted_polygon, &record)?;
    println!("Made shapefile: {:?}", start.elapsed());
    Ok(())
}
