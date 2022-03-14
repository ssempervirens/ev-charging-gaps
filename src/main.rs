use clap::Parser;
use std::error::Error;

use ev_charging_gaps::*;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Path to charger csv file
    #[clap(short, long)]
    path: String,
    /// Grid resolution, in degrees.
    #[clap(short, long, default_value_t = 0.01)]
    resolution: f64,
}

// fn download_source_data() -> Result<AllChargerLocations, Box<dyn Error>> {
//     let url = "https://developer.nrel.gov/api/alt-fuel-stations/v1.csv?access=public&api_key=oMa5C8ffgw2DXGNv7HHaWSZKWx2rGeBGdkfLvL70&cards_accepted=all&cng_fill_type=all&cng_psi=all&cng_vehicle_class=all&country=all&download=true&e85_has_blender_pump=false&ev_charging_level=2%2Cdc_fast&ev_connector_type=all&ev_network=all&fuel_type=ELEC&hy_is_retail=true&limit=all&lng_vehicle_class=all&lpg_include_secondary=false&offset=0&owner_type=all&state=all&status=E&utf8_bom=true";
//     let body = reqwest::blocking::get(url)?.text()?;
//     let mut reader = Reader::from_reader(body.as_bytes());
//     let rows = reader
//         .deserialize()
//         .filter_map(|row: Result<ChargerLocation, _>| row.ok())
//         .map(|location| {
//             let id = ItemId(location.id as usize);
//             let point = Item::Point(Point {
//                 x: location.latitude as f32,
//                 y: location.longitude as f32,
//             });
//             chargers_by_id.insert(id, location);
//             (id, point)
//         });
//     let quadtree = QuadTree::new(rows);
//     println!(
//         "tree = {:?}; len = {}",
//         quadtree.bbox(),
//         chargers_by_id.len()
//     );
//     Ok(AllChargerLocations {
//         quadtree,
//         chargers_by_id,
//     })
//     Ok(())
// }

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    // Bounding box of continental United States
    let lat_start = 49.1756;
    let lon_start = -124.7580;
    let lat_end = 24.5243;
    let lon_end = -66.9472;

    let charger_locations = download_source_data()?;
    // let charger_locations = read_csv(&args.path)?;

    let grid = generate_grid(args.resolution, lat_start, lon_start, lat_end, lon_end);
    let total = grid.len();
    println!("generated grid (length: {})", total);
    let mut reachable = 0;
    let mut unreachable = 0;
    let mut maybe_reachable = 0;
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
            }
        }
        if i % 1_000 == 0 {
            println!("{}: {:?}", i, start.elapsed());
        }
    }
    println!(
        "Resolution: {}\n\nTotal points: {}\nReachable: {}\nUnreachable: {}\nUnknown: {}",
        args.resolution, total, reachable, unreachable, maybe_reachable
    );
    Ok(())
}
