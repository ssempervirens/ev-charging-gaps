use clap::Parser;
use reqwest::blocking::Client;
use std::error::Error;

use ev_charging_gaps::*;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Path to charger csv file
    #[clap(short, long)]
    path: Option<String>,
    /// Grid resolution, in degrees.
    #[clap(short, long, default_value_t = 0.01)]
    resolution: f64,
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
        None => download_source_data(),
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
    for (i, point) in grid.into_iter().enumerate() {
        let result = point.check_charger(&charger_locations);
        match result {
            CheckResult::Yes => {
                reachable += 1;
            }
            CheckResult::No => {
                unreachable += 1;
            }
            CheckResult::Maybe { candidates } => {
                maybe_reachable += 1;
                // Find the distance between points and chargers that are maybe reachable
                // Where candidates is a vector of ChargerLocations
                let mut is_reachable = false;
                for (charger, _) in candidates {
                    let distance = point.get_osrm_distance(&client, &charger) as u64;
                    api_call_counter += 1;
                    if distance <= MAX_RANGE_METERS {
                        reachable += 1;
                        is_reachable = true;
                        break;
                    }
                }
                if is_reachable == false {
                    unreachable += 1;
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
    Ok(())
}
