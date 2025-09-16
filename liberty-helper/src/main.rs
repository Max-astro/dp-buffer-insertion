use liberty_db::{DefaultCtx, Library, PinId};

fn get_kb(p1: (f64, f64), p2: (f64, f64)) -> (f64, f64) {
    let k = (p2.1 - p1.1) / (p2.0 - p1.0);
    let b = p1.1 - k * p1.0;
    (k, b)
}

fn get_arc_info(
    lib: &Library<DefaultCtx>,
    cell_name: &str,
    ipin: &str,
    opin: &str,
) -> (f64, f64, f64) {
    let trans = 0.01;
    let loading = [0.01, 0.15];

    // println!("{} {} {}", cell_name, ipin, opin);
    let cell = lib.cell.get(cell_name).unwrap();
    let timings = cell
        .pin
        .get(&PinId::from(opin))
        .map(|pin| &pin.timing)
        .unwrap();

    for arc in timings.iter() {
        if arc.related_pin == PinId::from(ipin).into() {
            let rise_points = arc
                .cell_rise
                .as_ref()
                .map(|tbl| {
                    (
                        tbl.lookup(&trans, &loading[0]).unwrap(),
                        tbl.lookup(&trans, &loading[1]).unwrap(),
                    )
                })
                .unwrap();
            let fall_points = arc
                .cell_fall
                .as_ref()
                .map(|tbl| {
                    (
                        tbl.lookup(&trans, &loading[0]).unwrap(),
                        tbl.lookup(&trans, &loading[1]).unwrap(),
                    )
                })
                .unwrap();

            let p1 = (loading[0], (rise_points.0 + fall_points.0) / 2.0);
            let p2 = (loading[1], (rise_points.1 + fall_points.1) / 2.0);
            let kb = get_kb(p1, p2);
            return (
                kb.0,
                kb.1,
                cell.pin
                    .get(&PinId::from(ipin))
                    .and_then(|pin| pin.capacitance)
                    .unwrap(),
            );
        }
    }

    panic!("Can't find cell {}'s arc: {} -> {}", cell_name, ipin, opin);
}

fn buffer_info(lib: &Library<DefaultCtx>) -> Vec<(f64, f64, f64)> {
    // for sky130
    let base_name = "sky130_fd_sc_hd__buf";
    let mut ret = vec![];

    for size in [1, 2, 4, 6, 8, 12, 16] {
        let cell_name = format!("{base_name}_{size}");
        ret.push(get_arc_info(lib, &cell_name, "A", "X"));
    }

    ret
}

fn inv_info(lib: &Library<DefaultCtx>) -> Vec<(f64, f64, f64)> {
    // for sky130
    let base_name = "sky130_fd_sc_hd__inv";
    let mut ret = vec![];

    for size in [1, 2, 4, 6, 8, 12, 16] {
        let cell_name = format!("{base_name}_{size}");
        ret.push(get_arc_info(lib, &cell_name, "A", "Y"));
    }

    ret
}

use plotly::color::Rgb;
use plotly::common::{Marker, Mode};
use plotly::layout::{Axis, Layout, Legend};
use plotly::{ImageFormat, Plot, Scatter};
// use std::env::args;
// use std::path::Path;
fn plot_buf_inv_slope(lib: &Library<DefaultCtx>) {
    let buffers = buffer_info(&lib);
    let invs = inv_info(&lib);
    let sizes = [1, 2, 4, 6, 8, 12, 16];

    let mut plot = Plot::new();

    let x: f64 = 0.15;
    for ((k, b, _c), sz) in buffers.iter().zip(sizes.iter()) {
        let y = *k * x + *b;
        let rise_trace = Scatter::new([0.0, x].to_vec(), [*b, y].to_vec())
            .name(format!("buf_{}", sz))
            .mode(Mode::Lines);
        plot.add_trace(rise_trace);
    }
    for ((k, b, _c), sz) in invs.iter().zip(sizes.iter()) {
        let y = *k * x + *b;
        let rise_trace = Scatter::new([0.0, x].to_vec(), [*b, y].to_vec())
            .name(format!("inv_{}", sz))
            .mode(Mode::Lines);
        plot.add_trace(rise_trace);
    }
    plot.show();
}

fn main() {
    // let file = args().nth(1).unwrap();
    // let file_path = Path::new(&file);
    // let lib = Library::<DefaultCtx>::parse_lib_file(file_path).unwrap();

    // let buffer_delay_functions = buffer_info(&lib);
    // let inv_delay_functions = inv_info(&lib);

    // let sizes = [1, 2, 4, 6, 8, 12, 16];

    // println!("In trans: 0.01");
    // println!("Loading: 0.01, 0.15\n");

    // // Format buffer delay functions
    // println!("Buffer sky130_fd_sc_hd__buf_* Info:");
    // println!("{:<8} {:<15} {:<15} {:<15}", "Size", "k", "b", "cap");
    // println!("{:-<50}", "");
    // for (i, (k, b, c)) in buffer_delay_functions.iter().enumerate() {
    // 	println!("{:<8} {:<15.6} {:<15.6} {:<15.6}", sizes[i], k, b, c);
    // }

    // println!();

    // // Format inverter delay functions
    // println!("Inverter sky130_fd_sc_hd__inv_* Info:");
    // println!("{:<8} {:<15} {:<15} {:<15}", "Size", "k", "b", "cap");
    // println!("{:-<50}", "");
    // for (i, (k, b, c)) in inv_delay_functions.iter().enumerate() {
    // 	println!("{:<8} {:<15.6} {:<15.6} {:<15.6}", sizes[i], k, b, c);
    // }

    // plot_buf_inv_slope(&lib);

    let sizes = vec![100, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000];
    let runtimes = vec![
        0.004078, 0.102442, 0.671875, 3.542880, 13.788697, 31.908673, 52.542079, 51.882666,
        160.426443, 271.935362,
    ];

    let mut plot = Plot::new();

    let line = Scatter::new(sizes.clone(), runtimes)
        .mode(Mode::LinesMarkers)
        .marker(Marker::new().size(8));
    plot.add_trace(line);

    let layout = Layout::new()
        // .title("Elapsed time vs. Fanouts")
        .x_axis(
            Axis::new()
                .title("Fanouts")
                .show_grid(false)
                .zero_line(false)
                .tick_values(sizes.into_iter().map(|x| x as f64).collect()),
        )
        .y_axis(Axis::new().title("Elapsed time (s)").zero_line(false).tick_values((0..=12).into_iter().map(|x| (x * 25) as f64).collect()));
    plot.set_layout(layout);

    // plot.show();
    // plot.write_html("runtime.html");
    plot.write_image("runtime.svg", ImageFormat::SVG, 800, 600, 1.0);
}
