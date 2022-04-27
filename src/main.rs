use clap::Parser;
use rayon::prelude::*;
use reqwest::blocking::Client;
use shapefile::dbase;
use shapefile::Multipoint;
use std::error::Error;
use std::sync::{
    atomic::{AtomicUsize, Ordering::Relaxed},
    Arc,
};
use std::time::Instant;

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
    let lat_min = 24.5243;
    let lon_min = -124.7580;
    let lat_max = 49.1756;
    let lon_max = -66.9472;
    let client = Client::new();

    let charger_locations = match args.path {
        Some(path) => read_from_file(&path),
        None => download_source_data(
            &args
                .nrel_api_key
                .expect("If there was no path provided, there should be a NREL API key"),
        ),
    }?;
    let cpus = dbg!(num_cpus::get() * 16);
    let bounding_box = BoundingBox {
        lat_min,
        lon_min,
        lat_max,
        lon_max,
    };
    let chunks = bounding_box.chunkify(cpus);
    let completed = Arc::new(AtomicUsize::new(0));
    let start = Instant::now();
    let polygons: Vec<_> = chunks
        .into_par_iter()
        .map_with(
            (charger_locations, args.osrm_url, completed),
            |(charger_locations, osrm_url, completed), c| {
                let start = Instant::now();
                let polygon =
                    charger_locations.find_gaps(args.resolution, c, &osrm_url, client.clone());
                println!(
                    "Completed chunk {}/{} in {:?}",
                    completed.fetch_add(1, Relaxed),
                    cpus,
                    start.elapsed()
                );
                polygon
            },
        )
        .collect();
    println!("Completed all chunks in {:?}", start.elapsed());
    let table_info = dbase::TableWriterBuilder::new()
        .add_logical_field(dbase::FieldName::try_from("has_charger").unwrap());
    let mut writer = shapefile::Writer::from_path("output/test_shapefile3.shp", table_info)?;
    let mut record = dbase::Record::default();
    record.insert(
        "has_charger".to_owned(),
        dbase::FieldValue::Logical(Some(false)),
    );
    let mut points = Vec::new();
    for mut p in polygons {
        points.append(&mut p);
    }
    let converted_multipoint = Multipoint::from(geo::MultiPoint(points));
    writer.write_shape_and_record(&converted_multipoint, &record)?;
    Ok(())
}
