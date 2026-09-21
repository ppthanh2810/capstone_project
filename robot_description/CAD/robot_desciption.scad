// AMR COMPLETE - mm
// +X forward | +Y left | +Z up | ground Oxy: Z=0

$fn=60;
part="assembly"; // assembly, caster, camera, wheel, base
show_frames=true;
show_track=false;

//========================= KÍCH THƯỚC CHÍNH =========================
ground_z=0;
base_d=400; base_t=5;
wheel_d=107; wheel_w=41.5; wheel_r=wheel_d/2;
wheel_axis_z=ground_z+wheel_r;
base_underside_z=wheel_axis_z+60;
z_base_bottom=base_underside_z+base_t/2;
z_base_top=z_base_bottom+105;
bottom_surface=z_base_bottom-base_t/2;
top_surface=z_base_top+base_t/2;
post_xy=[[120,100],[-120,100],[-120,-100],[120,-100]];

//========================= VỊ TRÍ PHẦN CỨNG =========================
bread_size=[170,63,10]; bread_xy=[110,0]; bread_yaw=90;
z_bread=top_surface+bread_size[2]/2;

imu_size=[21,12,1.6];
imu_xyz=[110,0,top_surface+bread_size[2]+imu_size[2]/2];

camera_size=[25,90,25];
camera_xyz=[170,0,top_surface+40+camera_size[2]/2];

lidar_post_xy=[[170,60],[170,-60]];
lidar_post_h=100; lidar_plate_t=3;
z_lidar_plate=top_surface+lidar_post_h+lidar_plate_t/2;
lidar_xyz=[150,0,z_lidar_plate+lidar_plate_t/2+41.3/2];

motor_block=[110,25,40]; motor_y=100;
z_motor=bottom_surface-motor_block[2]/2;

plate_size=[50,10,80]; plate_y=117.5;
z_plate=bottom_surface-plate_size[2]/2;

wheel_track=340; wheel_y=wheel_track/2; wheel_x=0;

//========================= BÁNH CHỦ ĐỘNG =========================
hub_overhang=9.25; hub_d=84; label_d=48;
bolt_pcd=72; bolt_d=3.5;
shaft_d=12; shaft_flat=10;
shaft_len=36.75-hub_overhang;
hub_outer=wheel_w/2+hub_overhang;
axle_hole_d=13;
wheel_top_z=wheel_axis_z+wheel_r;
wheel_bottom_z=wheel_axis_z-wheel_r;

//========================= HÀM HỖ TRỢ =========================
module cylY(d,h,y=0)
    translate([0,y,0])
        rotate([90,0,0])
            cylinder(d=d,h=h,center=true);

module cylX(d,h)
    rotate([0,90,0])
        cylinder(d=d,h=h,center=true);

module rrbox(s,r=2.5)
    hull()
        for(x=[-1,1],y=[-1,1],z=[-1,1])
            translate([
                x*(s[0]/2-r),
                y*(s[1]/2-r),
                z*(s[2]/2-r)
            ])
                sphere(r);

module arrow(L=20,r=.7){
    cylinder(h=L-4,r=r);
    translate([0,0,L-4]) cylinder(h=4,r1=2*r,r2=0);
}

module frame_xyz(L=20){
    color("red")   rotate([0,90,0])  arrow(L);
    color("green") rotate([-90,0,0]) arrow(L);
    color("blue")  arrow(L);
}

//========================= CHASSIS VÀ IMU =========================
module base()
    cylinder(d=base_d,h=base_t,center=true);

module imu(){
    color("darkblue") cube(imu_size,center=true);
    color("black") translate([0,0,1.3]) cube([5.2,3.8,1.1],center=true);
}

//========================= REALSENSE D435 =========================
module pillX(w,h,t)
    hull()
        for(y=[-(w-h)/2,(w-h)/2])
            translate([0,y,0])
                cylX(h,t);

module camera_lens(y,d,glass="#18232d",rainbow=false){
    color("#777e83") translate([13.27,y,0]) cylX(d+1.2,.30);
    color("#07090b") translate([13.46,y,0]) cylX(d,.42);
    color(glass) translate([13.71,y,0]) cylX(d*.65,.17);
    color(rainbow?"#37ada5":"#718d9d")
        translate([13.82,y-.8,.8]) cylX(d*.14,.10);
}

module camera_d435(){
    // Chỉ thay hình dạng; hộp bao vẫn 25(X) × 90(Y) × 25(Z) mm.
    color("#9ca0a3") pillX(90,25,21);
    color("#bdc0c2") pillX(88,23,25);
    color("#e1e3e4") translate([12.57,0,0]) pillX(87.8,21.8,.55);
    color("#121316") translate([12.93,0,0]) pillX(84.5,18.7,.65);

    camera_lens(-32,9.2,"#1b2834");
    camera_lens(-11,11,"#14635d",true);
    camera_lens(18,9.2,"#29353e");
    camera_lens(35,5.5,"#252529");

    color("#25e351") translate([13.39,23,-7.4]) cylX(1.2,.35);
    for(y=[-27:4:27])
        color("#51565a") translate([0,y,12.53]) cube([8,1.15,.12],center=true);
    color("#191b1d") translate([-12.58,-32,0]) cube([.25,8,3],center=true);
    color("#3c3f41") translate([0,0,-12.53]) cylinder(d=6.35,h=.14,center=true);
}

//========================= GIÁ CAMERA THEO ẢNH =========================
// Đường bao tối đa: Ø23 × 40 mm
// Đáy tại top_surface, đỉnh đúng mặt dưới camera.

module camera_mount_photo(){
    color([.67,.69,.71]){
        // Trụ dưới
        translate([0,0,9])
            cylinder(d=23,h=18,center=true);

        // Vòng cổ
        translate([0,0,19.5])
            cylinder(d=21,h=3,center=true);

        // Cổ côn
        translate([0,0,24])
            cylinder(d1=18,d2=14,h=6,center=true);

        // Khớp chữ U
        translate([0,0,31])
            difference(){
                rrbox([14,16,14],2);
                translate([0,0,4])
                    cube([16,7,10],center=true);
            }

        // Chốt ngang
        translate([0,0,34])
            cylY(5,18);

        // Chốt bắt vào camera
        translate([0,0,38.5])
            cylinder(d=6,h=3,center=true);
    }

    // Hai đầu chốt
    for(s=[-1,1])
        color("#3b3b3b")
            translate([0,s*9.2,34])
                cylY(7,1.2);
}

//========================= RPLIDAR =========================
module rplidar(){
    color("black")
        translate([0,0,-13])
            linear_extrude(height=15,center=true)
                offset(r=5)
                    square([45.6,45.6],center=true);

    color("#222222")
        translate([0,0,7.5])
            cylinder(d=48,h=26.3,center=true);

    color("#444444")
        translate([0,0,20.8])
            cylinder(d=34,h=1,center=true);
}

module lidar_plate()
    linear_extrude(height=lidar_plate_t,center=true)
        polygon([
            [110,-85],
            [180,-85],
            [200,0],
            [180,85],
            [110,85]
        ]);

//========================= MOTOR VÀ BÁNH =========================
module motor_plate(){
    difference(){
        cube(plate_size,center=true);

        translate([wheel_x,0,wheel_axis_z-z_plate])
            rotate([90,0,0])
                cylinder(
                    d=axle_hole_d,
                    h=plate_size[1]+2,
                    center=true
                );
    }
}

module dshaft(L)
    intersection(){
        rotate([90,0,0])
            cylinder(d=shaft_d,h=L,center=true);

        cube([shaft_flat,L+1,shaft_d+1],center=true);
    }

module drive_wheel_local(){
    color("#111111")
        cylY(wheel_d,wheel_w);

    for(s=[-1,1])
        color(s<0 ? "#181818" : "silver")
            cylY(
                hub_d,
                hub_overhang,
                s*(wheel_w/2+hub_overhang/2)
            );

    color("#1769c2")
        cylY(label_d,.8,-hub_outer-.4);

    for(a=[0:60:300])
        color("silver")
            translate([
                bolt_pcd/2*cos(a),
                -hub_outer-.7,
                bolt_pcd/2*sin(a)
            ])
                cylY(bolt_d,1);

    color("silver")
        cylY(20,2,hub_outer+1);

    color("silver")
        translate([0,hub_outer+shaft_len/2,0])
            dshaft(shaft_len);
}

module drive_wheel(side)
    if(side<0)
        drive_wheel_local();
    else
        rotate([0,0,180])
            drive_wheel_local();

module wheel_nut(side)
    color("silver")
        translate([
            0,
            side*(plate_y+plate_size[1]/2+3),
            wheel_axis_z
        ])
            rotate([90,0,0])
                cylinder(
                    d=22/cos(30),
                    h=6,
                    center=true,
                    $fn=6
                );

//========================= CASTER Ø75 × 30 =========================
c_mount_x=120;
c_plate=[55,75,3];

c_frus_top=34;
c_frus_bot=40;
c_frus_h=7;
c_ring_d=40;
c_ring_h=2;

c_body_L=50;
c_body_H=48;
c_wall=1.5;
c_inside_Y=32;
c_outside_Y=c_inside_Y+2*c_wall;
c_x_front=-36;
c_x_rear=12;

c_wheel_d=75;
c_wheel_w=30;
c_arm_t=2.5;
c_arm_w=11;
c_axle_d=10;
c_pivot_d=10;
c_mid_d=8;

c_mount_z=bottom_surface-c_plate[2]/2;
c_z_plate_bot=-c_plate[2]/2;
c_z_frus_bot=c_z_plate_bot-c_frus_h;
c_z_ring_bot=c_z_frus_bot-c_ring_h;
c_z_top=c_z_ring_bot;
c_z_bot=c_z_top-c_body_H;

c_pivot=[c_x_rear-3,c_z_bot+4];
c_wheel=[-52,ground_z+c_wheel_d/2-c_mount_z];
c_mid=(c_pivot+c_wheel)/2;
c_arm_y=c_wheel_w/2+2+c_arm_t/2;

c_spring_x=-7;
c_spring_y=8;
c_spring_bottom=c_z_bot+7;
c_spring_top=c_z_top-7;

module c_torus(R,r)
    rotate_extrude()
        translate([R,0])
            circle(r=r,$fn=18);

module c_nut(af=14,h=5,hole=10.6)
    difference(){
        cylY(af/cos(30),h);
        cylY(hole,h+1);
    }

module c_link(a,b,w)
    hull(){
        translate(a) circle(d=w);
        translate(b) circle(d=w);
    }

module c_swivel(){
    color("silver")
        cube(c_plate,center=true);

    color("gainsboro")
        translate([0,0,c_z_frus_bot])
            cylinder(
                h=c_frus_h,
                d1=c_frus_bot,
                d2=c_frus_top
            );

    color([.78,.65,.16])
        translate([0,0,c_z_ring_bot])
            cylinder(d=c_ring_d,h=c_ring_h);
}

module c_profile()
    polygon([
        [c_x_front+2,c_z_top],
        [c_x_rear+8,c_z_top],
        [c_x_rear+8,c_z_top-14],
        [c_x_rear+6,c_z_top-23],
        [c_x_rear+3,c_z_top-32],
        [c_x_rear,c_z_bot+10],
        [c_x_rear-3,c_z_bot+2],
        [c_x_front-1,c_z_bot],
        [c_x_front-7,c_z_bot+10],
        [c_x_front-8,c_z_bot+23],
        [c_x_front-6,c_z_top-13]
    ]);

module c_housing(){
    for(s=[-1,1])
        color("silver")
            translate([
                0,
                s*(c_outside_Y/2-c_wall/2),
                0
            ])
                rotate([90,0,0])
                    linear_extrude(
                        height=c_wall,
                        center=true
                    )
                        c_profile();

    color("silver")
        translate([
            (c_x_front+c_x_rear)/2,
            0,
            c_z_top-c_wall/2
        ])
            cube(
                [c_body_L,c_outside_Y,c_wall],
                center=true
            );

    color("silver")
        translate([-8,0,c_z_bot+c_wall/2])
            cube([24,c_outside_Y,c_wall],center=true);
}

module c_spring(y){
    color("silver")
        translate([
            c_spring_x,
            y,
            c_spring_bottom-2
        ])
            cylinder(
                d=4,
                h=c_spring_top-c_spring_bottom+6
            );

    color([.43,.28,.14])
        for(z=[
            c_spring_bottom+2.5
            :
            3
            :
            c_spring_top-2.5
        ])
            translate([c_spring_x,y,z])
                c_torus(3.3,.5);
}

module c_springs(){
    for(y=[-c_spring_y,c_spring_y])
        c_spring(y);

    color("silver")
        translate([c_spring_x,0,c_z_bot+2])
            cylY(6,c_outside_Y+5);
}

module c_arm2d(){
    c_link(c_wheel,c_pivot,c_arm_w);
    translate(c_wheel) circle(d=c_arm_w+4);
    translate(c_pivot) circle(d=c_arm_w+4);
}

module c_fork(){
    for(y=[-c_arm_y,c_arm_y])
        color("silver")
            translate([0,y,0])
                rotate([90,0,0])
                    linear_extrude(
                        height=c_arm_t,
                        center=true
                    )
                        c_arm2d();

    color("silver"){
        translate([c_pivot[0],0,c_pivot[1]])
            cylY(c_pivot_d,c_outside_Y+9);

        translate([c_mid[0],0,c_mid[1]])
            cylY(c_mid_d,2*c_arm_y+4);

        translate([c_wheel[0],0,c_wheel[1]])
            cylY(c_axle_d,2*c_arm_y+8);
    }

    for(s=[-1,1]){
        color("gainsboro")
            translate([
                c_pivot[0],
                s*(c_outside_Y/2+4),
                c_pivot[1]
            ])
                c_nut(hole=c_pivot_d+.6);

        color("gainsboro")
            translate([
                c_wheel[0],
                s*(c_arm_y+4),
                c_wheel[1]
            ])
                c_nut(hole=c_axle_d+.6);
    }
}

module c_wheel_model(){
    color([.28,.28,.28])
        hull()
            for(s=[-1,1]){
                translate([
                    0,
                    s*(c_wheel_w/2-.4),
                    0
                ])
                    cylY(c_wheel_d-7,.8);

                translate([
                    0,
                    s*(c_wheel_w/2-3),
                    0
                ])
                    cylY(c_wheel_d,.8);
            }

    for(s=[-1,1]){
        color([.8,.8,.82])
            translate([
                0,
                s*(c_wheel_w/2+.6),
                0
            ])
                cylY(52,1.2);

        color("gainsboro")
            translate([
                0,
                s*(c_wheel_w/2+1.4),
                0
            ])
                cylY(24,1.8);
    }
}

module caster(){
    c_swivel();
    c_housing();
    c_springs();
    c_fork();

    translate([c_wheel[0],0,c_wheel[1]])
        c_wheel_model();
}

module two_casters(){
    translate([c_mount_x,0,c_mount_z])
        caster();

    translate([-c_mount_x,0,c_mount_z])
        rotate([0,0,180])
            caster();
}

//========================= LẮP RÁP =========================
module assembly(){
    color("gray")
        translate([0,0,z_base_bottom])
            base();

    color("silver")
        translate([0,0,z_base_top])
            base();

    for(p=post_xy)
        color("gold")
            translate([
                p[0],
                p[1],
                z_base_bottom+52.5
            ])
                cylinder(d=12,h=100,center=true);

    color("white")
        translate([
            bread_xy[0],
            bread_xy[1],
            z_bread
        ])
            rotate([0,0,bread_yaw])
                cube(bread_size,center=true);

    translate(imu_xyz)
        rotate([0,0,bread_yaw])
            imu();

    // Giá camera mới: đáy tại top_surface, cao đúng 40 mm
    translate([
        camera_xyz[0],
        camera_xyz[1],
        top_surface
    ])
        camera_mount_photo();

    // Camera giữ nguyên camera_xyz
    translate(camera_xyz)
        camera_d435();

    for(p=lidar_post_xy)
        color("steelblue")
            translate([
                p[0],
                p[1],
                top_surface+50
            ])
                cylinder(d=12,h=100,center=true);

    color([.7,.85,1,.65])
        translate([0,0,z_lidar_plate])
            lidar_plate();

    translate(lidar_xyz)
        rplidar();

    for(side=[-1,1]){
        color("white")
            translate([0,side*motor_y,z_motor])
                cube(motor_block,center=true);

        color("silver")
            translate([0,side*plate_y,z_plate])
                motor_plate();

        translate([
            wheel_x,
            side*wheel_y,
            wheel_axis_z
        ])
            drive_wheel(side);

        wheel_nut(side);
    }

    two_casters();

    if(show_track)
        for(s=[-1,1])
            color("red")
                translate([0,s*wheel_y,wheel_axis_z])
                    cube([280,1,1],center=true);

    if(show_frames){
        translate([0,0,ground_z])
            frame_xyz(30);

        translate(imu_xyz)
            frame_xyz(16);

        translate(camera_xyz)
            frame_xyz(18);

        translate(lidar_xyz)
            frame_xyz(18);
    }

    echo("GROUND Z",ground_z);
    echo("MAIN WHEEL BOTTOM Z",wheel_bottom_z);
    echo("CASTER BOTTOM Z",
        c_mount_z+c_wheel[1]-c_wheel_d/2);

    echo("BASE UNDERSIDE Z",bottom_surface);

    echo(
        "CAMERA CENTER Z (UNCHANGED)",
        camera_xyz[2]
    );

    echo(
        "CAMERA MOUNT TOP Z",
        top_surface+40
    );

    echo(
        "COMMON AXES",
        "+X forward, +Y left, +Z up"
    );
}

//========================= HIỂN THỊ =========================
if(part=="assembly")
    assembly();

else if(part=="caster")
    caster();

else if(part=="camera"){
    camera_mount_photo();
    translate([0,0,52.5])
        camera_d435();
}

else if(part=="wheel")
    drive_wheel(-1);

else if(part=="base")
    base();
