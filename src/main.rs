use clap::Parser;
use reqwest::blocking::Client;
use shapefile::dbase;
use shapefile::Polygon;
use std::error::Error;
use std::thread;

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
    let client = Client::new();

    let charger_locations = match args.path {
        Some(path) => read_from_file(&path),
        None => download_source_data(
            &args
                .nrel_api_key
                .expect("If there was no path provided, there should be a NREL API key"),
        ),
    }?;
    let cpus = dbg!(num_cpus::get());
    let bounding_box = BoundingBox {
        lat_start,
        lon_start,
        lat_end,
        lon_end,
    };
    let chunks = bounding_box.chunkify(cpus);
    let mut join_handle_vec = Vec::new();
    for c in chunks {
        let charger_locations = charger_locations.clone();
        let osrm_url = args.osrm_url.clone();
        let client = client.clone();
        let thread_join_handle = thread::spawn(move || {
            charger_locations.find_gaps(args.resolution, c, &osrm_url, client)
        });
        join_handle_vec.push(thread_join_handle);
    }
    let table_info = dbase::TableWriterBuilder::new()
        .add_logical_field(dbase::FieldName::try_from("has_charger").unwrap());
    let mut writer = shapefile::Writer::from_path("output/test_shapefile.shp", table_info)?;
    let mut record = dbase::Record::default();
    record.insert(
        "has_charger".to_owned(),
        dbase::FieldValue::Logical(Some(false)),
    );
    for join_handle in join_handle_vec {
        let res = join_handle.join().unwrap();
        let converted_polygon = Polygon::from(res);
        writer.write_shape_and_record(&converted_polygon, &record)?;
    }
    Ok(())
}
