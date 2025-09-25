module top(
    \1_1_a[0] ,
    \1_1_a[1] ,
    \1_1_a[2] ,
    \1_1_a[3] ,
    \1_1_a[4] ,
    \1_1_a[5] ,
    \1_1_a[6] ,
    \1_1_a[7] ,
    \1_1_a[8] ,
    \1_1_a[9] ,
    \2_2_remainder[0] ,
    \2_2_remainder[1] ,
    \2_2_remainder[2] ,
    \2_2_remainder[3] ,
    \2_2_remainder[4] ,
    \2_2_remainder[5] ,
    \2_2_remainder[6] ,
    \2_2_remainder[7] ,
    \2_2_remainder[8] ,
    \2_2_remainder[9]
);
    input \1_1_a[0] ;
    input \1_1_a[1] ;
    input \1_1_a[2] ;
    input \1_1_a[3] ;
    input \1_1_a[4] ;
    input \1_1_a[5] ;
    input \1_1_a[6] ;
    input \1_1_a[7] ;
    input \1_1_a[8] ;
    input \1_1_a[9] ;
    output \2_2_remainder[0] ;
    output \2_2_remainder[1] ;
    output \2_2_remainder[2] ;
    output \2_2_remainder[3] ;
    output \2_2_remainder[4] ;
    output \2_2_remainder[5] ;
    output \2_2_remainder[6] ;
    output \2_2_remainder[7] ;
    output \2_2_remainder[8] ;
    output \2_2_remainder[9] ;

    wire N_00, N_01, N_02, N_03, N_04;

    sky130_fd_sc_hd__nand2_1 C_269 ( .A(\1_1_a[0] ), .B(\1_1_a[1] ), .Y(N_00) );
    sky130_fd_sc_hd__nand3_4 C_270 ( .A(\1_1_a[2] ), .B(\1_1_a[3] ), .C(\1_1_a[4] ), .Y(N_01) );
    sky130_fd_sc_hd__nand3b_1 C_271 ( .A(\1_1_a[5] ), .B(\1_1_a[6] ), .C(\1_1_a[7] ), .Y(N_02) );
    sky130_fd_sc_hd__bufinv_8 C_272 ( .A(\1_1_a[8] ), .Y(N_03) );
    sky130_fd_sc_hd__nand2_1 C_273 ( .A(\1_1_a[9] ), .B(N_03), .Y(N_04) );

    sky130_fd_sc_hd__nand2_1 C_274 ( .A(N_01), .B(N_01), .Y(\2_2_remainder[0] ) );
    sky130_fd_sc_hd__nand2_1 C_275 ( .A(N_00), .B(N_02), .Y(\2_2_remainder[1] ) );
    sky130_fd_sc_hd__nand3_4 C_41 ( .A(N_01), .B(N_03), .C(N_04), .Y(\2_2_remainder[2] ) );
    sky130_fd_sc_hd__nand3_4 C_42 ( .A(N_02), .B(N_04), .C(\1_1_a[9] ), .Y(\2_2_remainder[3] ) );
    sky130_fd_sc_hd__nand3b_1 C_43 ( .A(N_03), .B(N_02), .C(\1_1_a[2] ), .Y(\2_2_remainder[4] ) );
    sky130_fd_sc_hd__nand4_1 C_44 ( .A(N_03), .B(N_04), .C(\1_1_a[3] ), .D(N_00), .Y(\2_2_remainder[5] ) );
    sky130_fd_sc_hd__bufinv_8 C_45 ( .A(N_04), .Y(\2_2_remainder[6] ) );
    sky130_fd_sc_hd__bufinv_8 C_46 ( .A(N_04), .Y(\2_2_remainder[7] ) );
    sky130_fd_sc_hd__nand2_1 C_59 ( .A(N_02), .B(\1_1_a[5] ), .Y(\2_2_remainder[8] ) );
    sky130_fd_sc_hd__nand2_1 C_60 ( .A(N_00), .B(N_01), .Y(\2_2_remainder[9] ) );
endmodule