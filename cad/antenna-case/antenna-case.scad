module antenna(){
    rotate([90,0,0])cylinder(h=22,r=2,$fn=100);
    translate([-23.2/2,0,-1])cube([23.2,23.2,3]);
    translate([-21/2,1,-2])cube([21,21,6.5]);
    translate([-21/2,1,-2])cube([21,22,3]);
    translate([-20/2,0,-2])cube([20,3,3]);
    translate([0,23.2/2,0])cylinder(h=4.8,r=7);
}


module caseb_outer() {
    rotate([90,0,0])cylinder(h=17,r=3.2,$fn=6);
    translate([-25/2,-1,-2.771])cube([25,25,2.771]);

    translate([-10,23.5,-1])cube([20,1,1]);
    translate([-10,-1.4,-1])cube([20,1,1]);
}
module caseb() difference(){
    caseb_outer();
    antenna();
    translate([-25,-25,0])cube([50,60,5]);
    
    translate([11.5,-2,-3])rotate([0,30,0])cube([5,30,5]);
    translate([-15.5,-2,-6])rotate([0,-30,0])cube([5,30,5]);
    
    translate([2.8,-17,-2])cube([10,3,4]);
    translate([-12.8,-17,-2])cube([10,3,4]);
}


module caset_outer(){
    rotate([90,0,0])cylinder(h=17,r=3.2,$fn=6);
    translate([-25/2,-1,0])cube([25,25,2.771]);

    translate([-24/2,-0.5,0])cube([24,24,5.2]);
    translate([-24.4/2,-0.7,0])cube([24.4,24.4,3]);
}

module caset_notabs() difference(){
    caset_outer();
    antenna();
    translate([-25,-25,-5])cube([50,60,5]);
}
module tabprofile() difference(){
    translate([-10,24.0,-2.5])cube([20,1.2,4.0]);
    translate([-10,23.5,-1.2])cube([20,1,1.2]);
    
    translate([-10,24.3,-2.6])rotate([-30,0,0])cube([20,1,2]);
    translate([-10,25.3,0.0])rotate([30,0,0])cube([20,1,2]);
}

module caset(){
    caset_notabs();
    tabprofile();
    difference(){
        translate([0,23,0])scale([1,-1,1]) tabprofile();
        translate([-3.5,-2.5,-2.6])cube([7,5,5]);
    }
    difference(){
        intersection(){
            translate([0,-14,0])rotate([90,0,0])cylinder(h=3,r=4.2,$fn=6);
            translate([-5,-20,-1.5])cube([10,10,4.271]);
        }
        translate([0,-14,0])rotate([90,0,0])cylinder(h=4,r=3.2,$fn=6);
        
    }
}

//difference(){
color("#8f8") caset();
  //  translate([0,-25,-10])cube([50,60,20]);
//}

//render()
//caseb();


difference(){
    //color("#f88")caseb();
    //translate([-25,10,-10])cube([50,60,20]);
}



%antenna();