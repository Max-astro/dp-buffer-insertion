use cxx::CxxString;
use liberty_db::timing::TimingTableLookUp;
use liberty_db::{Cell, DefaultCtx, Library, PinId};

pub struct LibDb {
    lib: Library<DefaultCtx>,
}

pub struct LibCell {
    cell: *const Cell<DefaultCtx>,
}

pub struct TimingTbl2D {
    tbl: *const TimingTableLookUp<DefaultCtx>,
}

#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type LibDb;
        type LibCell;
        type TimingTbl2D;

        fn read_lib(file: &CxxString) -> Box<LibDb>;
        fn get_cell(lib: &LibDb, cell_name: &CxxString) -> Box<LibCell>;
        fn get_pin_capacitance(cell: &LibCell, pin: &CxxString) -> f64;
        fn get_timing_table(
            cell: &LibCell,
            ipin: &CxxString,
            opin: &CxxString,
            rise: bool,
        ) -> Box<TimingTbl2D>;
        fn get_transition_table(
            cell: &LibCell,
            ipin: &CxxString,
            opin: &CxxString,
            rise: bool,
        ) -> Box<TimingTbl2D>;
        fn lookup(tbl: &TimingTbl2D, trans: f64, loading: f64) -> f64;
    }
}

fn read_lib(file: &CxxString) -> Box<LibDb> {
    let binding = file.to_str().expect("invalid file name");
    let path = std::path::Path::new(binding);
    let lib = Library::<DefaultCtx>::parse_lib_file(path).expect("failed to parse liberty file");
    Box::new(LibDb { lib })
}

fn get_cell(lib: &LibDb, cell_name: &CxxString) -> Box<LibCell> {
    let name = cell_name.to_str().expect("invalid cell name");
    let cell = lib.lib.cell.get(name);
    assert!(cell.is_some(), "cell not found: {}", name);
    Box::new(LibCell {
        cell: cell.unwrap() as *const _,
    })
}

fn get_pin_capacitance(cell: &LibCell, pin: &CxxString) -> f64 {
    let pin = pin.to_str().expect("invalid pin name");
    let lib_ref = unsafe { &*cell.cell };
    let pin_ref = lib_ref.pin.get(&PinId::from(pin)).expect("pin not found");
    pin_ref.capacitance.unwrap()
}

fn get_timing_table(
    cell: &LibCell,
    ipin: &CxxString,
    opin: &CxxString,
    rise: bool,
) -> Box<TimingTbl2D> {
    let ipin = ipin.to_str().expect("invalid input pin name");
    let opin = opin.to_str().expect("invalid output pin name");
    let lib_ref = unsafe { &*cell.cell };
    let timings = lib_ref
        .pin
        .get(&PinId::from(opin))
        .map(|pin| &pin.timing)
        .expect("pin not found or no timing");

    for arc in timings.iter() {
        if arc.related_pin == PinId::from(ipin).into() {
            let tbl = if rise {
                arc.cell_rise.as_ref()
            } else {
                arc.cell_fall.as_ref()
            };
            let rf = if rise { "rise" } else { "fall" };
            assert!(
                tbl.is_some(),
                "cell {} doens't have {} timing table",
                lib_ref.name,
                rf
            );
            let tbl = tbl.unwrap();
            return Box::new(TimingTbl2D {
                tbl: tbl as *const _,
            });
        }
    }

    panic!(
        "LibCell {} timing arc not found: {} -> {}",
        lib_ref.name, ipin, opin
    );
}

fn get_transition_table(
    cell: &LibCell,
    ipin: &CxxString,
    opin: &CxxString,
    rise: bool,
) -> Box<TimingTbl2D> {
    let ipin = ipin.to_str().expect("invalid input pin name");
    let opin = opin.to_str().expect("invalid output pin name");
    let lib_ref = unsafe { &*cell.cell };
    let timings = lib_ref
        .pin
        .get(&PinId::from(opin))
        .map(|pin| &pin.timing)
        .expect("pin not found or no timing");

    for arc in timings.iter() {
        if arc.related_pin == PinId::from(ipin).into() {
            let tbl = if rise {
                arc.rise_transition.as_ref()
            } else {
                arc.fall_transition.as_ref()
            };
            let rf = if rise { "rise" } else { "fall" };
            assert!(
                tbl.is_some(),
                "cell {} doens't have {} transition table",
                lib_ref.name,
                rf
            );
            let tbl = tbl.unwrap();
            return Box::new(TimingTbl2D {
                tbl: tbl as *const _,
            });
        }
    }

    panic!(
        "LibCell {} timing arc not found: {} -> {}",
        lib_ref.name, ipin, opin
    );
}

fn lookup(tbl: &TimingTbl2D, trans: f64, loading: f64) -> f64 {
    let tbl = unsafe { &*tbl.tbl };
    tbl.lookup(&trans, &loading)
        .expect("timing table lookup failed")
}
