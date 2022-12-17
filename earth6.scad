//
//	OpenSCAD model of earth showing distance and connections between locations
//
//	Uses 6 locations on the earth and connects each
//	location with the others for a total of 12 connections.
//
//	The locations and connections between locations are chosen such that:
//		Locations are well known, popular destinations
//		Each location has 4 connections to the others
//		All 'faces' formed by the connections are triangles
//		No connection crosses another
//
//	Using Euler's polyhedra formula (V-E+F=2) with V=6, E=12 we find F=8 and
//	thus there are 8 triangular faces on the rendered sphere which form
//	a crude octahedron.
//
//	The variable 'Globe' can be true or false.  When true, locations
//	and connections are rendered on a sphere.  When false, locations
//	are mapped to a plane using an azimuthal equidistant (AE) projection.
//	The AE projection is scaled to give the equator a radius of Earth_radius.
//
//	All distances in kilometers.
//
//	OpenSCAD is somewhat odd in that trig functions expect arguments in degrees
//	and inverse trig functions return degrees.
//
//	OpenSCAD is NOT a programming language. It cannot do simple things like sum a
//	list of distances.  Thus the AE distance between two points is difficult to calculate.
//	Uncomment the 'echo' statements in ae_connection() to get a rough estimate when Globe=false.
//
//	Formulas based on examples at https://www.movable-type.co.uk/scripts/latlong.html
//

Globe		= true;		// globe or flat model
References	= false;	// render reference globe/disc
Steps		= 51;		// number of steps to use when Globe=false (set to 1 for shortest distance)
Thick		= 500;		// drawn thickness of connections

module dummy(){}	// customizer stop

$fn=128;
Earth_radius	= 6371.0710;	// Google maps.  WGS84 would be 6378.1370
Flattening	= 0.0;		// Google maps.  WGS84 would be 1.0/298.257223563;
Proud		= 1.15;		// lettering is raised by this multiplier (1.0=none)
Ball_thick	= Thick*2;	// locations are much bigger
Ref_thick	= Thick/4;	// reference arcs/lines are thinner

LAT=0; LON=1; TAG=2;	// crude 'struct'
Locations = [
	[  40.689236,  -74.044577, "SOL" ],	// New York, USA (Statue of Liberty)
	[  27.175083,   78.042119, "TJM" ],	// Agra, India (Taj Mahal)
	[  21.364877, -157.950010, "AZM" ],	// Honolulu, USA (Arizona WWII Memorial)
	[ -33.856726,  151.214983, "SOH" ],	// Sydney, Australia (Sydney Opera House)
	[ -34.357125,   18.473748, "CGH" ],	// Capetown, South Africa (Cape of Good Hope)
	[ -54.846834,  -68.480849, "EOW" ],	// Ushuaia, Argentina (End of the World Post Office)
];

SOL=0; TJM=1; AZM=2; SOH=3; CGH=4; EOW=5;	// keep in sync with Locations[]
Connections = [
	[SOL,AZM],
	[SOL,EOW],
	[TJM,SOL],
	[TJM,CGH],
	[AZM,TJM],
	[AZM,SOH],
	[SOH,TJM],
	[SOH,CGH],
	[CGH,SOL],
	[CGH,EOW],
	[EOW,AZM],
	[EOW,SOH],
];

// helper functions
function dtor(d)	= (d*PI)/180;				// degrees to radians
function sin2(a)	= sin(a) * sin(a);			// sin squared
function cos2(a)	= cos(a) * cos(a);			// cos squared
function lon(a)		= atan2(a.y,a.x);			// degrees longitude of a point [x,y,z]
function lat(a)		= asin(a.z/norm(a));			// degrees latitude of a point [x,y,z]
function angle(a,b)	= acos((a*b)/(norm(a)*norm(b)));	// angle in degrees between 2 points [x,y,z]
function distance(a,b)	= norm(a-b);				// straight-line distance between 2 points [x,y,z]

// Half-reversed sine: distance along an arc between 2 points [x,y,z], returned distance is on unit sphere
function h_base(a,b) = sin2((lat(b)-lat(a))/2) + (cos(lat(a)) * cos(lat(b))) * sin2((lon(b)-lon(a))/2);
function haversine(a,b) = 2 * dtor(atan2( sqrt(h_base(a,b)) , sqrt(1-h_base(a,b))));

// bearing from a to b
function bearing(a,b) = atan2(sin(lon(b)-lon(a)) * cos(lat(b)),
	cos(lat(a))*sin(lat(b)) - sin(lat(a))*cos(lat(b))*cos(lon(b)-lon(a)));

// convert a point [x,y,z] to its AE 2D position [x,y,0] (center = north pole)
function ae_map(a) =
	[ dtor(90-lat(a)) * norm(a)/PI*2 * cos(lon(a)),
	  dtor(90-lat(a)) * norm(a)/PI*2 * sin(lon(a)),
	  0 ];

// calculate point on sphere between a and b  (fract=[0..1])
// a and b can be any 2 points [x,y,z], returned point is on unit sphere
function pb_A(q,fract) = sin((1-fract)*q) / sin(q);
function pb_B(q,fract) = sin(    fract*q) / sin(q);
function pbq(a) = 2 * atan2(sqrt(a), sqrt(1-a));
function pba(a,b) = sin2((lat(b)-lat(a))/2) + cos(lat(a)) * cos(lat(b)) * sin2((lon(b)-lon(a))/2);
function point_between(a,b,fract) =
	[ (pb_A(pbq(pba(a,b)),fract) * cos(lat(a)) * cos(lon(a))) + (pb_B(pbq(pba(a,b)),fract) * cos(lat(b)) * cos(lon(b))),
	  (pb_A(pbq(pba(a,b)),fract) * cos(lat(a)) * sin(lon(a))) + (pb_B(pbq(pba(a,b)),fract) * cos(lat(b)) * sin(lon(b))),
	  (pb_A(pbq(pba(a,b)),fract) * sin(lat(a))              ) + (pb_B(pbq(pba(a,b)),fract) * sin(lat(b))             )];

// convert [lat,lon] to [x,y,z] returned point is on unit sphere
// If Flattening is non-zero, apply correction factor for better accuracy
Flat2 = (1.0-Flattening)*(1.0-Flattening);	// (1-Flattening) squared
function xyz_base(lat) = 1/sqrt(cos2(lat) + (Flat2 * sin2(lat)));
function xyz(a) =
	[ xyz_base(a[LAT]) * cos(a[LAT]) * cos(a[LON]),
	  xyz_base(a[LAT]) * cos(a[LAT]) * sin(a[LON]),
	  xyz_base(a[LAT]) * sin(a[LAT]) * Flat2 ];

// a torus of large radius r1, small radius r2
module torus(r1,r2)
{
	rotate_extrude(convexity = 1)
		translate([r1, 0, 0])
			circle(r2);
}

// a cylinder between 2 [x,y,z] points with radius r
module rod(a, b, r)
{
	dir = b-a;
	h  = norm(dir);
	w  = dir / h;
	u0 = cross(w, [0,0,1]);
	u  = u0 / norm(u0);
	v0 = cross(w, u);
	v  = v0 / norm(v0);

	color("lightblue")
	multmatrix(m=[  [u.x, v.x, w.x, a.x],
			[u.y, v.y, w.y, a.y],
			[u.z, v.z, w.z, a.z],
			[  0,   0,   0,   1]])
		cylinder(r=r, h=h);
}

// a great circle arc of 0-180 degrees
// center of arc will be at [radius,0,0] and aligned along the Z-axis
// add text 'label' to the middle of the arc
module great_circle_arc(degrees,radius,label)
{
	difference(){
		color("lightblue")
		difference() {
			torus(radius,Thick);

			// trim top half
			rotate([0,0,degrees/2])
			translate([-radius*1.5,0,-radius*1.5])
				cube (radius*3);

			// trim bottom half
			rotate([0,0,-degrees/2])
			translate([-radius*1.5,-radius*3,-radius*1.5])
				cube (radius*3);
			}

		color("black")
		translate([radius+(Thick/2),0,0])
			rotate([90,0,90])
            linear_extrude(height=Thick)
				text(label,size=Thick*1.5,halign="center",valign="center");
		}
}

// a great circle arc that goes from point a to point b
module arc(a,b,radius,label)
{
	c = (a+b)/2;		// center directly between a,b (c is not on the sphere)
	rotate([90-bearing(c,a),-lat(c),lon(c)])
		great_circle_arc( angle(a,b), radius, label );
}

// draw a ball with a label
module ball(radius,label)
{
	difference(){
		color("green")
			sphere(radius);
		color("black")
            translate([0,0,radius-(Thick)])
                linear_extrude(height=radius)
				text(label,size=radius*0.60,halign="center",valign="center");
		
	}
}

module earth_location(pa)
{
	a = Earth_radius * xyz(pa);
	rotate([0,-lat(a),lon(a)])	// rotate to make the TAG face outward
	translate([Earth_radius,0,0])
	rotate([90,0,90])
		ball(Ball_thick,pa[TAG]);
}

module ae_location(pa)
{
	a = Earth_radius * xyz(pa);
	translate(ae_map(a))
		ball(Ball_thick,pa[TAG]);
}

module earth_connection(pa,pb)
{
	a = Earth_radius * xyz(pa);
	b = Earth_radius * xyz(pb);
	dist = Earth_radius * haversine(a,b);
	arc(a,b,Earth_radius,str(round(dist)));
	//echo("From",pa[TAG],"To",pb[TAG],dist);
}

module ae_connection(pa,pb)
{
	for(i=[0:Steps-1]){
		a = Earth_radius * point_between(xyz(pa),xyz(pb),i/Steps);
		b = Earth_radius * point_between(xyz(pa),xyz(pb),(i+1)/Steps);
		dist = distance(ae_map(a),ae_map(b));
		//echo("From",pa[TAG],"To",pb[TAG],i,dist);
		rod(ae_map(a),ae_map(b),Thick);
		color("lightblue")translate(ae_map(b))sphere(Thick);
		}
}

module globe_ref()
{
	%sphere(Earth_radius);					// reference sphere
	color("red")torus(Earth_radius,Ref_thick); 	 	// equator
	color("red")rotate([90,0,0])torus(Earth_radius,Ref_thick); // meridian
	color("red")rotate([0,90,0])torus(Earth_radius,Ref_thick); // meridian
	color("white")location([90,0,"NP"]);			// north pole
	color("white")location([-90,0,"SP"]);			// south pole
}

module ae_ref()
{
	aer = Earth_radius*2;

	%circle(aer);						// reference disc
	color("red")torus(aer/2,Ref_thick);			// equator
	color("red")rod([aer,0,0],[-aer,0,0],Ref_thick);	// meridian
	color("red")rod([0,aer,0],[0,-aer,0],Ref_thick);	// meridian
	color("white")ae_location([90,0,"NP"]);			// north pole
	color("white")torus(aer,Thick);				// outer edge
}

// one location
module location(pa)
{
	if(Globe)
		earth_location(pa);
	else
		ae_location(pa);
}

// one connection
module connection(pa,pb)
{
 	if(Globe)
		earth_connection(pa,pb);
	else
		ae_connection(pa,pb);
}

// reference
module refs()
{
	if(Globe)
		globe_ref();
	else
		ae_ref();
}

if(References)refs();
for(i=Locations) location(i);				// render locations
for(i=Connections) connection(Locations[i[0]],Locations[i[1]]);	// render connections
