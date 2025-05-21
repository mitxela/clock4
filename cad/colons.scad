$fn=100;

h=34.4;//34.1;
w=12.0;
d=10.5;

t_side=1;
t_front=1;
t_top=1.5;

tabwidth=5;
tabheight=2.9;

led_height=3;
led_dia=3.15;

ldr_height=1.9;

// colon positions
deltaV=11.3;
deltaH=1.85;

c1x= w/2-deltaH/2;
c2x= w/2+deltaH/2;
c1y= h/2+deltaV/2;
c2y= h/2-deltaV/2;

//latch size, depth and margin
ls=5.5;
ld=0.6;
lmv=0.15;
lmh=0.6;

module latchMale(){
    translate([w/2-ls/2,-ld,d/2-ls/2])cube([ls,ld,ls]);
}
module latchFemale(){
    //difference(){
    //    translate([0,-ld,0])cube([w,ld,d]);
        translate([w/2-ls/2-lmh,-ld*2,d/2-ls/2-lmv])cube([ls+lmh*2,ld*2,ls+lmv*2]);
    //}
}

module sideCutout(){
    sc_r = 7;
    sc_i = 12;
    translate([-1,sc_i,11])rotate([0,90,0])cylinder(h=w+2,r=sc_r);
    translate([-1,h-sc_i,11])rotate([0,90,0])cylinder(h=w+2,r=sc_r);
    translate([-1,sc_i,11])rotate([0,90,0])cube([sc_r,h-sc_i*2,w+2]);
}

module colon(){
    difference(){
        union(){
            difference() {
                cube([w,h,d]);
                translate([t_side,t_top,t_front])cube([w-t_side*2,h-t_top*2,d]);

                sideCutout();
            }
            translate([c1x,c1y,0])cylinder(led_height,d=5);
            translate([c2x,c2y,0])cylinder(led_height,d=5);
            tab();
            translate([w,h,0])rotate([0,0,180]) tab();
        }
        translate([c1x,c1y,-1])cylinder(11,d=led_dia);
        translate([c2x,c2y,-1])cylinder(11,d=led_dia);
    }
}

module switchcover(){
    h=34.6;
    difference(){
        union(){
            difference() {
                cube([w,h,d]);
                translate([t_side,t_top,t_front])cube([w-t_side*2,h-t_top*2,d]);
            }
            tab();
            translate([w,h,0])rotate([0,0,180]) tab();
            latchMale();
        }
        translate([w/2,h/2-4.5,-1])cylinder(11,d=3.5);
        translate([w/2,h/2+4.5,-1])cylinder(11,d=3.5);
        translate([0,-ld,0])magnet();
        translate([w/2-2,0.3,6])rotate([-20,0,0])cube([4,3,4]);
    }
    translate([0,-ld,0])magnetHolder();
    
    //latchMale();
    //latchFemale();
}

module magnet(){
    translate([w/2,4.0,d/2])rotate([90,0,0])cylinder(h=3.5,d=4.1);
}

module magnetHolder(){
    cutaway=0;
    difference(){
        translate([w/2-3,1,d/2-4.5])cube([6,3.5-cutaway,5]);
        magnet();
    }
    translate([w/2-3,4,5.75])rotate([-20,0,0])cube([6,0.5,2]);
    translate([w/2+2.1,3.22,5.5])rotate([-45,0,0])cube([0.9,0.5,1.5]);
    translate([w/2-3,3.22,5.5])rotate([-45,0,0])cube([0.9,0.5,1.5]);
}
ldr_pad=0.07;
module ldr(){
    h=34.6;
    difference(){
        union(){
            difference() {
                cube([w,h,d]);
                translate([t_side,t_top,t_front])cube([w-t_side*2,h-t_top*2,d]);
            }
            tab();
            translate([w,h,0])rotate([0,0,180]) tab();
            translate([w/2-2.5,h/2+1.45,0.5]){
                cube([5.0,1.0,2]);
                cube([5.0,1.3,1.6]);
            }
            translate([w/2-2.5,h/2-2.45,0.5])cube([5.0,1.0,2]);
            translate([w/2-2.5,h/2-2.75,0.5])cube([5.0,1.3,1.6]);
            
        }
        translate([w/2,h/2,-1]) intersection(){
            cylinder(ldr_height+1,r=2.5+ldr_pad);
            translate([-5,-4.0/2-ldr_pad,0])cube([10,4.0+ldr_pad*2,ldr_height+1]);
        }
        translate([w-1,1.5+3.58,d-2])cube([1,8,2]);
        translate([0,ld,0])magnet();
        //translate([w/2-2,0.9+ld,6])rotate([-20,0,0])cube([4,3,4]);
        translate([0,ld,0])latchFemale();
    }
    magnetHolder();

}

module tab(){
    difference(){
        translate([w/2-tabwidth/2,0,d]) {
            cube([tabwidth,1.5,tabheight-1]);
            translate([0,0,1.6])cube([tabwidth,2,tabheight-1-1.6]);
            translate([0,1.0,tabheight-1])rotate([0,90,0])cylinder(h=tabwidth,r=1.0);
        }
        translate([0,1.5,d]) cube([w,2,1.6]);
    }
}

module ldrFitTest() {
    cutaway=1;
    intersection(){
        ldr();
        translate([1,12,0]) cube([10-cutaway*5,10,5]);
    }
}


/*
intersection(){
    //ldr();
    switchcover();

    translate([0,-2,0])cube([w/2,40,20]);
}
//*/

//ldrFitTest();

//ldr();
//switchcover();
colon();

//translate([-40,0,0]) colon();
//translate([-20,0,0]) colon();
//translate([0,0,0]) switchcover();
//translate([20,0,0]) ldr();

