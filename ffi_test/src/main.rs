extern "C" {
    fn hello_world();
    fn execDWA(x: i32,
               y: i32,
               theta: i32,
               Vx: i32,
               Vy: i32,
               omega: i32,
               targetX: * mut i32,
               targetY: * mut i32,
               targetTheta: * mut i32,
               middle_targetX: * mut i32,
               middle_targetY: * mut i32,
               numberOfObstacle: * mut i32,
               ObstacleX: *mut i32,
               ObstacleY: *mut i32,
               ObstacleVX: *mut i32,
               ObstacleVY: *mut i32,
               prohibited_zone_ignore: bool,
               middle_target_flag: * mut bool,
               is_enable: * mut bool,
               path_enable: * mut bool,
               prohibited_zone_start: * mut bool,
               vx_out: * mut i32,
               vy_out: * mut i32,
               omega_out: * mut i32,
               ax_out: * mut i32,
               ay_out: * mut i32);
}

fn main() {
    println!("Hello, world! in Rust");
    unsafe {
        hello_world();
    }
    let x = 1;
    let y = 2;
    let theta = 3;
    let v_x = 4;
    let v_y = 5;
    let omega = 6;
    let mut target_x = 7;
    let mut target_y = 8;
    let mut target_theta = 9;
    let mut mid_tx = 10;
    let mut mid_ty = 11;
    let mut n_oo = 12;
    let mut obstacle_x = Vec::with_capacity(n_oo as usize);
    let mut obstacle_y = Vec::with_capacity(n_oo as usize);
    let mut obstacle_vx = Vec::with_capacity(n_oo as usize);
    let mut obstacle_vy = Vec::with_capacity(n_oo as usize);
    let pzi = false;
    let mut middle_target_flag = false;
    let mut is_enable = false;
    let mut path_enable = false;
    let mut pzs = false;
    let mut vx_out = 13;
    let mut vy_out = 14;
    let mut omega_out = 15;
    let mut ax_out = 16;
    let mut ay_out = 17;

    // println!("{}, {}", target_x, target_y);
    // println!("{}, {}", vx_out, ay_out);
    unsafe {
        execDWA(x, y, theta, v_x, v_y, omega, &mut target_x, &mut target_y, &mut target_theta, &mut mid_tx, &mut mid_ty, &mut n_oo, obstacle_x.as_mut_ptr(), obstacle_y.as_mut_ptr(), obstacle_vx.as_mut_ptr(), obstacle_vy.as_mut_ptr(), pzi, &mut middle_target_flag, &mut is_enable, &mut path_enable, &mut pzs, &mut vx_out, &mut vy_out, &mut omega_out, &mut ax_out, &mut ay_out);
        obstacle_x.set_len(n_oo as usize);
        obstacle_y.set_len(n_oo as usize);
        obstacle_vx.set_len(n_oo as usize);
        obstacle_vy.set_len(n_oo as usize);
    }
    println!("{}, {}", target_x, target_y);
    println!("{}, {}", vx_out, ay_out);
}
