//============================================================================
// Name        : obserbation-probability-position-tracker.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <time.h>

#include <ssmtype/spur-odometry.h>
#include <ssm.hpp>
#include <ssm-log.hpp>
#include "ssm-laser.hpp"

#include "opsm-position-tracker-offline-opt.hpp"
#include "opsm-position-tracker-offline-viewer.hpp"

#include "gnd-coord-tree.hpp"
#include "gnd-matrix-coordinate.hpp"
#include "gnd-matrix-base.hpp"

#include "gnd-observation-probability.hpp"
#include "gnd-odometry-rse-map.hpp"
#include "gnd-cui.hpp"
#include "gnd-gridmap.hpp"
#include "gnd-shutoff.hpp"
#include "gnd-time.hpp"
#include "gnd-bmp.hpp"


static const struct gnd::CUI::command cui_cmd[] = {
		{"Quit",					'Q',	"localizer shut-off"},
		{"help",					'h',	"show help"},
		{"show",					's',	"state show mode"},
		{"freq",					'f',	"change cycle (frequency)"},
		{"cycle",					'c',	"change cycle (cycle)"},
		{"start",					't',	"start (end stand-by mode)"},
		{"stand-by",				'B',	"stand-by mode"},
		{"viewer",					'v',	"viewer on / off"},
		{"", '\0'}
};

static const double ShowCycle = gnd_sec2time(1.0);
static const double ClockCycle = gnd_sec2time(1.0) / 60.0 ;

int main(int argc, char* argv[]) {
	gnd::opsm::optimizer_basic	*optimizer = 0;	// optimizer class
	void 						*starting = 0;	// optimization starting value

	gnd::opsm::counting_map_t	cnt_map;		// observation probability counting map
	gnd::opsm::map_t			op_map;			// observation probability map

	SSMLogBase					ssmlog_sokuikiraw;	// sokuiki raw streaming data
	void*						sokuikiraw_buf;
	ssm::ScanPoint2D			sokuikiraw;
	ssm::ScanPoint2DProperty	sokuikiraw_prop;
	SSMLog<Spur_Odometry>		ssmlog_odm;			// odometry position streaming data
//	SSMLog<Spur_Odometry>		ssmlog_pos;			// tracking result position streaming data

	gnd::matrix::coord_tree coordtree;			// coordinate tree
	int coordid_gl = -1,						// global coordinate node id
		coordid_rbt = -1,						// robot coordinate node id
		coordid_sns = -1,						// sensor coordinate node id
		coordid_odm = -1;						// odometry coordinate node id

	gnd::cui gcui;								// cui class

	gnd::rse_map emap;

	OPSMPosTrack::proc_configuration param;	// configuration parameter
	OPSMPosTrack::options proc_opt(&param);		// process option analyze class

	FILE *fp = 0;

	{
		gnd::opsm::debug_set_level(2);
		gnd::opsm::debug_set_fstream("debug.log");
	}


	{ // ---> initialization
		int ret;								// function return value
		size_t phase = 1;						// initialize phase



		// ---> read process options
		if( (ret = proc_opt.get_option(argc, argv)) != 0 ) {
			return ret;
		} // <--- read process options



		{ // ---> coordinate-tree set robot coordinate
			gnd::coord_matrix cc; // coordinate relation matrix

			// set global coordinate
			gnd::matrix::set_unit(&cc);
			coordid_gl = coordtree.add("global", "root", &cc);

			// set robot coordinate
			gnd::matrix::set_unit(&cc);
			coordid_rbt = coordtree.add("robot", "global", &cc);

			// set odometry coordinate
			gnd::matrix::set_unit(&cc);
			coordid_odm = coordtree.add("odometry", "root", &cc);	// local dead-reckoning

		} // <--- coordinate-tree set robot coordinate



		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off



		{ // ---> show initialize task
			::fprintf(stderr, "==========Initialize==========\n");
			::fprintf(stderr, " %d. create optimizer class \"%s\"", phase++, param.optimizer.value);
			if( param.slam.value && *param.mapdir.value ) {
				::fprintf(stderr, " %d. Map Data Load\n", phase++);
				::fprintf(stderr, " %d. Build %sMap\n", phase++, param.ndt.value ? "NDT " : "");
			}
			::fprintf(stderr, " %d. Open ssm-log \"\x1b[4m%s\x1b[0m\"\n", phase++, param.ls_logname.value);
			::fprintf(stderr, " %d. Open ssm-log \"\x1b[4m%s\x1b[0m\"\n", phase++, param.odm_logname.value);
			::fprintf(stderr, " %d. Initialize viewer\n", phase++);
			::fprintf(stderr, "\n\n");
		} // <--- show initialize task


		// ---> set optimizer
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create optimizer class \"%s\"\n", param.optimizer.value);
			if( !::strcmp(param.optimizer.value, OPSMPosTrack::OptNewton) ){
				optimizer = new gnd::opsm::newton;
				optimizer->create_starting_value(&starting);
				optimizer->set_converge_threshold(param.converge_dist.value,
						gnd_deg2ang( param.converge_orient.value ) );
				::fprintf(stderr, " ... newton's method \x1b[1mOK\x1b[0m\n");
			}
			else if( !::strcmp(param.optimizer.value, OPSMPosTrack::OptQMC)){
				gnd::opsm::qmc::starting_value *p;
				optimizer = new gnd::opsm::qmc;
				optimizer->create_starting_value(&starting);
				p = static_cast<gnd::opsm::qmc::starting_value*>(starting);
				p->n = 2;
				starting = static_cast<void*>(p);
				optimizer->set_converge_threshold(param.converge_dist.value,
						gnd_deg2ang( param.converge_orient.value ) );
				::fprintf(stderr, " ... quasi monte calro method \x1b[1mOK\x1b[0m\n");
			}
			else if( !::strcmp(param.optimizer.value, OPSMPosTrack::OptQMC2Newton)){
				gnd::opsm::hybrid_q2n::starting_value *p;
				optimizer = new gnd::opsm::hybrid_q2n;
				optimizer->create_starting_value(&starting);
				p = static_cast<gnd::opsm::hybrid_q2n::starting_value*>(starting);
				p->n = 2;
				starting = static_cast<void*>(p);
				optimizer->set_converge_threshold(param.converge_dist.value,
						gnd_deg2ang( param.converge_orient.value ) );
				::fprintf(stderr, " ... quasi monte calro and newton hybrid \x1b[1mOK\x1b[0m\n");
			}
			else {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid optimizer type\n");
			}

		} // ---> set optimizer



		// ---> map data load
		if( !::is_proc_shutoff() && param.slam.value && *param.mapdir.value ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Map Data Load\n");
			if( gnd::opsm::read_counting_map(&cnt_map, param.mapdir.value) < 0){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to map data\n");
			}
			else {
				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- map data load



		// ---> build map
		if( !::is_proc_shutoff() && param.slam.value && *param.mapdir.value ){
			::fprintf(stderr, "\n");
			if( !param.ndt.value){
				::fprintf(stderr, " => Build Map\n");
				if( gnd::opsm::build_map(&op_map, &cnt_map, 10, 1, 0.5) < 0) {
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build scan matching map\n");
				}
				else {
					::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
				}
			}
			else {
				::fprintf(stderr, " => Build NDT Map\n");
				if(gnd::opsm::build_ndt_map(&op_map, &cnt_map, gnd_mm2dist(1)) < 0){
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build scan matching map\n");
				}
				else {
					::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
				}
			}
		} // <--- build map

		// set map
		if(!::is_proc_shutoff() )	optimizer->set_map(&op_map);



		// ---> open ssm odometry
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Create ssm-data \"\x1b[4m%s\x1b[0m\"\n", param.odm_logname.value);
			if( !(*param.odm_logname.value) ){
				// shut off
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: missing log file operand\n" );
				::fprintf(stderr, "     please show help, ./%s -S <sokuiki.log> -O <odometry.log>\n", OPSMPosTrack::proc_name );
			}
			else {
				::fprintf(stderr, "    File \"\x1b[4m%s\x1b[0m\"\n", param.odm_logname.value);

				if( !ssmlog_odm.open( param.odm_logname.value ) ){
					::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", param.odm_logname.value);
				}
				else {
					::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", param.odm_logname.value);
					ssmlog_odm.readNext();

					{ // ---> set coordinate
						gnd::matrix::fixed<4,4> pos_cc;

						// odometry coordinate
						gnd::matrix::coordinate_converter(&pos_cc,
								ssmlog_odm.data().x, ssmlog_odm.data().y, 0,
								::cos(ssmlog_odm.data().theta), ::sin(ssmlog_odm.data().theta), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_odm, &pos_cc);
					} // ---> set coordinate
				}
			}
		} // <--- open ssm odometry



		// ---> open ssm sokuiki raw data
		if(!::is_proc_shutoff()){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", param.ls_logname.value);

			if( ! *param.ls_logname.value ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: missing log file operand\n" );
				::fprintf(stderr, "     please show help, ./%s -S <sokuiki.log> -O <odometry.log>\n", OPSMPosTrack::proc_name );
			}
			else {
				::fprintf(stderr, "    File \"\x1b[4m%s\x1b[0m\"\n", param.odm_logname.value);

				if( !ssmlog_sokuikiraw.open( param.ls_logname.value ) ){
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open ssm-data \"\x1b[4m%s\x1b[0m\"\n", param.ls_logname.value);
				}
				// get property
				else {
					// read property
					ssmlog_sokuikiraw.setBuffer(0, 0, &sokuikiraw_prop, sizeof(sokuikiraw_prop) );
					ssmlog_sokuikiraw.readProperty();

					// allocate memory
					sokuikiraw.alloc( sokuikiraw_prop.numPoints );
					sokuikiraw_buf = (void*) malloc( sokuikiraw._ssmSize() );

					ssmlog_sokuikiraw.setBuffer(sokuikiraw_buf, sokuikiraw._ssmSize(), &sokuikiraw_prop, sizeof(sokuikiraw_prop) );
					ssmlog_sokuikiraw.readNext();
                    ssm::ScanPoint2D::_ssmRead( ssmlog_sokuikiraw.data(), &sokuikiraw, 0);

					{ // ---> coordinate-tree set sensor coordinate
						coordid_sns = coordtree.add("sensor", "robot", &sokuikiraw_prop.coordm);
					} // <--- coordinate-tree set robot coordinate
					::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
				}
			}
		} // <--- open ssm sokuiki raw data



		// ---> viewer initialization
		if ( !::is_proc_shutoff() ) {
			gnd::gl::initialize(&argc, argv);

			gnd::gl::window.m = GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE;
			gnd::gl::window.w = 400;
			gnd::gl::window.h = 300;

			gnd::gl::window.field = 10;
			::strcpy( gnd::gl::window.name, "viewer" );

			gnd::gl::window.df = OPSMPosTrack::Viewer::display;
			gnd::gl::window.idle = OPSMPosTrack::Viewer::idle;
			gnd::gl::window.mf = OPSMPosTrack::Viewer::mouse;
			gnd::gl::window.reshf = OPSMPosTrack::Viewer::reshape;

			if ( param.debug_viewer.value ) gnd::gl::begin();
		} // <--- viewer initialization


		if ( !::is_proc_shutoff() ) {
			if( !(fp = fopen("odm-error.log.txt", "w")) ) {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", "odm-error.log.txt");
			}
			else {
                ::fprintf(fp, "# 1.[time, s] 2.[x] 3.[y] 4.[theta] 5.[v] 6.[w]. 7.[adjust-x] 8.[adjust-y] 9.[adjust-theta] 10.[adjust-v] 11.[adjust-w].\n");
			}
		}

		if ( !::is_proc_shutoff() ) {
			// create map
			gnd::odometry::rsemap::create(&emap, param.pos_gridsize.value, param.ang_rsl.value);
		}


		// set cui command
		gcui.set_command(cui_cmd, sizeof(cui_cmd) / sizeof(cui_cmd[0]));


		// fin of initialization
		::fprintf(stderr, "\n\n");
	} // <--- initialization












	// ---> operation
	if(!::is_proc_shutoff() ){
		int ret = 0;									// function return value
		int cnt = 0;

		Spur_Odometry odo_prevloop = ssmlog_odm.data(); // previous odometry position
		Spur_Odometry move_est;							// estimation of movement quantity

		double decimate_sqdist							// data decimation threshold
			= gnd_square( param.decimate.value );
		double lkl = 0;									// likelihood
		double lkl_prev_opt = 0;						// previous likelihood
		Spur_Odometry pos_opt;							// optimized position
		int opt_cnt = 0;								// optimization loop counter

		gnd::matrix::fixed<3,1> move_opt;				// position estimation movement by optimization
		double prev_time = ssmlog_odm.time();
		double running = 0;

		double init_time = ssmlog_odm.time();
		Spur_Odometry pos = ssmlog_odm.data();
		Spur_Odometry pos_premap = ssmlog_odm.data();	// odometry position streaming data
		ssmTimeT time_premap = 0;						// previous map update time

		double sqdist_mapup =							// distance threshold of map update
				gnd_square( param.rest_dist.value );

		gnd::matrix::fixed<4,4> coordm_sns2rbt;			// coordinate convert matrix from sensor to robot
		gnd::matrix::fixed<4,4> coordm_sns2gl;				// coordinate convert matrix from sensor to global

		double cuito = 0;								// blocking time out for cui input

		gnd::Time::IntervalTimer timer_show;			// time operation timer

		bool map_update = false;

		// get coordinate convert matrix
		coordtree.get_convert_matrix(coordid_sns, coordid_rbt, &coordm_sns2rbt);

		{ // ---> set zero
			move_est.x = 0;
			move_est.y = 0;
			move_est.theta = 0;
			move_est.v = 0;
			move_est.w = 0;
		} // <--- set zero



		// ---> memory allocate counting map
		if( !cnt_map.plane[0].is_allocate() ){
			gnd::opsm::init_counting_map(&cnt_map, 0.5, 10);
		} // <--- memory allocate counting map


		{ // ---> map initialization
			int cnt_ls = 0;

			::fprintf(stderr, "-------------------- map initialize  --------------------\n");
			// ---> map initialization loop
			while( !::is_proc_shutoff() && cnt_ls < param.ini_map_cnt.value ) {

				// ---> read ssm-sokuikiraw-data
				if(ssmlog_sokuikiraw.readNext()) {
					{ // ---> 1. compute position estimation from odometry
						// get position on odometry position (on odometry cooordinate)
						if( !ssmlog_odm.readTime( ssmlog_sokuikiraw.time() ) ) continue;

	                    ssm::ScanPoint2D::_ssmRead( ssmlog_sokuikiraw.data(), &sokuikiraw, 0);

						{ // ---> compute the movement estimation
							gnd::matrix::fixed<4,1> odov_cprev;			// current odometry position vector on previous odometry coordinate

							{ // ---> compute current odometry position on previous odometry coordinate
								gnd::matrix::fixed<4,4> coordm_r2podo;		// coordinate matrix of previous odometry position
								gnd::matrix::fixed<4,1> ws4x1;

								// get previous odometry coordinate matrix
								coordtree.get_convert_matrix(0, coordid_odm, &coordm_r2podo);

								// multiply previous odometry coordinate matrix with current position vector
								ws4x1[0][0] = ssmlog_odm.data().x;
								ws4x1[1][0] = ssmlog_odm.data().y;
								ws4x1[2][0] = 0;
								ws4x1[3][0] = 1;
								gnd::matrix::prod(&coordm_r2podo, &ws4x1, &odov_cprev);
							} // <--- compute current odometry position on previous odometry coordinate

							// get movement estimation by odometry
							move_est.x = odov_cprev[0][0];
							move_est.y = odov_cprev[1][0];
							move_est.theta = ssmlog_odm.data().theta - odo_prevloop.theta;
						} // <--- compute the movement estimation


						{ // ---> add movement estimation
							gnd::matrix::fixed<4,1> pos_odmest;

							{ // ---> compute position estimation by odometry on global coordinate
								gnd::matrix::fixed<4,4> coordm_rbt2gl;		// coordinate convert matrix from robot to global
								gnd::matrix::fixed<4,1> ws4x1;

								// set search position on sensor-coordinate
								coordtree.get_convert_matrix(coordid_rbt, coordid_gl, &coordm_rbt2gl);

								ws4x1[0][0] = move_est.x;
								ws4x1[1][0] = move_est.y;
								ws4x1[2][0] = 0;
								ws4x1[3][0] = 1;

								gnd::matrix::prod(&coordm_rbt2gl, &ws4x1, &pos_odmest);
							} // <--- compute position estimation by odometry on global coordinate

							// set position
							pos.x = pos_odmest[0][0];
							pos.y = pos_odmest[1][0];
							pos.theta += move_est.theta;
						} // <--- add movement estimation
					}  // <--- 1. compute position estimation from odometry


					{ // ---> 2. update robot position coordinate and odometory position coordinate
						gnd::matrix::fixed<4,4> coordm;

						// odometry coordinate
						gnd::matrix::coordinate_converter(&coordm,
								ssmlog_odm.data().x, ssmlog_odm.data().y, 0,
								::cos(ssmlog_odm.data().theta), ::sin(ssmlog_odm.data().theta), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_odm, &coordm);

						// robot position coordinate
						gnd::matrix::coordinate_converter(&coordm,
								pos.x, pos.y, 0,
								::cos( pos.theta ), ::sin( pos.theta ), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_rbt, &coordm);

						// get coordinate convert matrix
						coordtree.get_convert_matrix(coordid_sns, coordid_gl, &coordm_sns2gl);
					} // ---> 2. update robot position coordinate and odometory position coordinate

					gnd::matrix::set_zero(&move_opt);
					{ // ---> 3. entry laser scanner reading
						gnd::matrix::fixed<3,1> delta;
						gnd::matrix::fixed<2,1> reflect_prevent;

						// clear previous entered sensor reading
						gnd::matrix::set_zero(&reflect_prevent);

						// ---> scanning loop for sokuikiraw-data
						for(size_t i = 0; i < sokuikiraw.numPoints(); i++){
							// ---> entry laser scanner reflection
							gnd::matrix::fixed<4,1> reflect_csns, reflect_cgl;
							gnd::matrix::fixed<3,1> ws3x1;
							gnd::matrix::fixed<3,3> ws3x3;

							// if range data is null because of no reflection
							if(sokuikiraw[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
							// ignore error data
							else if(sokuikiraw[i].isError()) 	continue;
							else if(sokuikiraw[i].r < sokuikiraw_prop.distMin * 1.1)	continue;
							else if(sokuikiraw[i].r > sokuikiraw_prop.distMax * 0.9)	continue;

							{ // ---> compute laser scanner reading position on robot coordinate
								// set search position on sensor-coordinate
								reflect_csns[0][0] = sokuikiraw[i].r * ::cos(sokuikiraw[i].th);
								reflect_csns[1][0] = sokuikiraw[i].r * ::sin(sokuikiraw[i].th);
								reflect_csns[2][0] = 0;
								reflect_csns[3][0] = 1;


								// data decimation with distance threshold
								if( gnd_square(reflect_csns[0][0] - reflect_prevent[0][0]) + gnd_square(reflect_csns[1][0] - reflect_prevent[1][0]) < decimate_sqdist ){
									continue;
								}
								else {
									// update previous entered data
									gnd::matrix::copy(&reflect_prevent, &reflect_csns);
								}

								// convert from sensor coordinate to robot coordinate
								gnd::matrix::prod(&coordm_sns2gl, &reflect_csns, &reflect_cgl);
							} // <--- compute laser scanner reading position on robot coordinate

							// data entry
							gnd::opsm::counting_map(&cnt_map, reflect_cgl[0][0], reflect_cgl[1][0]);
						} // <--- scanning loop for sokuikiraw-data
					} // <--- 3. entry laser scanner reading


					odo_prevloop = ssmlog_odm.data();
					cnt_ls++;
					::fprintf(stderr, ".");
				} // <--- read ssm sokuikiraw
			} // <--- map initialization loop

			// ---> map build
			if( gnd::opsm::build_map(&op_map, &cnt_map, 10, 1.0e-3, 0) < 0 ){
				::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
			}
			else {
				::fprintf(stderr, "\n... \x1b[1mOK\x1b[0m\n");
			} // <--- map build

		} // <--- map initialization




		{ // ---> timer
			// set parameter-cycle
//			timer_operate.begin(CLOCK_REALTIME, param.cycle.value, -param.cycle.value);
//			timer_clock.begin(CLOCK_REALTIME,
//					param.cycle.value < ClockCycle ? param.cycle.value :  ClockCycle);
			if( param.debug_show.value )	timer_show.begin(CLOCK_REALTIME, ShowCycle, -ShowCycle);
			else 							::fprintf(stderr, "  > ");
		} // <--- timer



		// ---> operation loop
		while ( !::is_proc_shutoff() ) {
//			timer_clock.wait();

			{ // ---> cui
				int cuival = 0;
				char cuiarg[512];
				// zero reset buffer
				::memset(cuiarg, 0, sizeof(cuiarg));

				// ---> get command
				if( gcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito) > 0 ){
					if( timer_show.cycle() > 0 ){
						// quit show status mode
						timer_show.end();
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					}
					else { // ---> cui command operation
						switch(cuival){
						// exit
						case 'Q': ::proc_shutoff(); break;
						// help
						default:
						case '\0':
						case 'h': gcui.show(stderr, "   "); break;
						// show status
						case 's': timer_show.begin(CLOCK_REALTIME, ShowCycle, -ShowCycle); break;
						case 'f': {
							double freq = ::strtod(cuiarg, 0);
							if( freq <= 0 ){
								::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid argument value (frequency 0)\n");
								::fprintf(stderr, "       if you want to stop estimator, send \"\x1b[4mstand-by\x1b[0m\" command\n");
							}
							else {
								double cyc = 1.0 / freq;
//								timer_operate.begin(CLOCK_REALTIME, cyc);
//								timer_clock.begin(CLOCK_REALTIME,
//										param.cycle.value < ClockCycle ? param.cycle.value :  ClockCycle);
								::fprintf(stderr, "   ... cycle %.03lf\n", cyc);
							}
						} break;

						// set freq
						case 'c': {
							double cyc = ::strtod(cuiarg, 0);
							if( cyc <= 0 ){
								::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid argument value (frequency 0)\n");
								::fprintf(stderr, "       if you want to stop estimator, send \"\x1b[4mstand-by\x1b[0m\" command\n");
							}
							else {
//								timer_operate.begin(CLOCK_REALTIME, cyc);
//								timer_clock.begin(CLOCK_REALTIME,
//										param.cycle.value < ClockCycle ? param.cycle.value :  ClockCycle);
								::fprintf(stderr, "   ... cycle %.03lf\n", cyc);
							}
						} break;

						// start
						case 't':{
							cuito = 0.0;
						} break;
						// stand-by
						case 'B':{
							::fprintf(stderr, "   stand-by mode\n");
							cuito = -1;
						} break;

						case 'v': {
							if ( !param.debug_viewer.value ) {
								::fprintf(stderr, "   ... create window\n");
								if( !gnd::gl::is_thread_fin() ) {
									::fprintf(stderr, "   ... \x1b[31m\x1b[1mFial\x1b[0m\x1b[39m can'not create because of thread is busy\n");
								}
								else if( gnd::gl::begin() < 0){
									::fprintf(stderr, "   ... \x1b[31m\x1b[1mFial\x1b[0m\x1b[39m can'not create because of thread is busy\n");
								}
								param.debug_viewer.value = true;
							}
							else if ( param.debug_viewer.value ) {
								::fprintf(stderr, "   ... delete\n");
								if( gnd::gl::end() < 0 ) {
									::fprintf(stderr, "   ... \x1b[31m\x1b[1mFial\x1b[0m\x1b[39m can'not delete window\n");
								}
								param.debug_viewer.value = false;
							}
						} break;
						}
					} // <--- cui command operation
					::fprintf(stderr, "  > ");
					gcui.poll(&cuival, cuiarg, sizeof( cuiarg ), 0);
				} // <--- get command
			}  // <--- cui


			// ---> show status
			if( timer_show.clock() > 0){
				::fprintf(stderr, "\x1b[0;0H\x1b[2J");	// display clear
				::fprintf(stderr, "-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", OPSMPosTrack::proc_name);
				::fprintf(stderr, "          loop : %d\n", cnt);
				::fprintf(stderr, " optimize loop : %d\n", opt_cnt);
				::fprintf(stderr, "    likelihood : %.03lf\n", lkl );
				::fprintf(stderr, "      position : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						pos.x, pos.y, gnd_ang2deg( pos.theta ) );
				::fprintf(stderr, "      optimize : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						move_opt[0][0], move_opt[1][0], gnd_ang2deg( move_opt[2][0] ) );
				::fprintf(stderr, "      move est : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						move_est.x, move_est.y, gnd_ang2deg( move_est.theta ) );
//				::fprintf(stderr, "      cycle : %.03lf\n", timer_operate.cycle() );
				::fprintf(stderr, "     optimizer : %s\n", ret == 0 ? "success" : "failure" );
				::fprintf(stderr, "    map update : %s\n", map_update ? "true" : "false" );
				::fprintf(stderr, "\n");
				::fprintf(stderr, " Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
			} // <--- show status

			// ---> read ssm-sokuikiraw-data
			if( ssmlog_sokuikiraw.readNext()) {
				// ---> position tracking
				// ... operation flow
				//      *0. get laser scanner reading
				//       1. compute position estimation by odometry (on global coordinate)
				//		 2. set position estimation by odometry to optimization starting value
				//		 3. optimization iteration by matching laser scanner reading to map(likelihood field)
				//		 4. optimization error test and write position ssm-data
				//		 5. robot current position coordinate and odometory position coordinate
				//		 6. update map and view data


				{  // ---> 1. compute position estimation from odometry
					double dt;
					// get position on odometry position (on odometry cooordinate)
					if( !ssmlog_odm.readTime(ssmlog_sokuikiraw.time()) ) continue;

					dt = ssmlog_odm.time() - prev_time;
					prev_time = ssmlog_odm.time();
					running += ssmlog_odm.data().v * dt;

					if( running * running < param.rest_dist.value * param.rest_dist.value  )
						continue;

                    ssm::ScanPoint2D::_ssmRead( ssmlog_sokuikiraw.data(), &sokuikiraw, 0);

					{ // ---> compute the movement estimation
						gnd::matrix::fixed<4,1> odov_cprev;			// current odometry position vector on previous odometry coordinate

						{ // ---> compute current odometry position on previous odometry coordinate
							gnd::matrix::fixed<4,4> coordm_r2podo;		// coordinate matrix of previous odometry position
							gnd::matrix::fixed<4,1> ws4x1;

							// get previous odometry coordinate matrix
							coordtree.get_convert_matrix(0, coordid_odm, &coordm_r2podo);

							// multiply previous odometry coordinate matrix with current position vector
							ws4x1[0][0] = ssmlog_odm.data().x;
							ws4x1[1][0] = ssmlog_odm.data().y;
							ws4x1[2][0] = 0;
							ws4x1[3][0] = 1;
							gnd::matrix::prod(&coordm_r2podo, &ws4x1, &odov_cprev);
						} // <--- compute current odometry position on previous odometry coordinate

						// get movement estimation by odometry
						move_est.x = odov_cprev[0][0];
						move_est.y = odov_cprev[1][0];
						move_est.theta = ssmlog_odm.data().theta - odo_prevloop.theta;
					} // <--- compute the movement estimation


					{ // ---> add movement estimation
						gnd::matrix::fixed<4,1> pos_odmest;

						{ // ---> compute position estimation by odometry on global coordinate
							gnd::matrix::fixed<4,4> coordm_rbt2gl;		// coordinate convert matrix from robot to global
							gnd::matrix::fixed<4,1> ws4x1;

							// set search position on sensor-coordinate
							coordtree.get_convert_matrix(coordid_rbt, coordid_gl, &coordm_rbt2gl);

							ws4x1[0][0] = move_est.x;
							ws4x1[1][0] = move_est.y;
							ws4x1[2][0] = 0;
							ws4x1[3][0] = 1;

							gnd::matrix::prod(&coordm_rbt2gl, &ws4x1, &pos_odmest);
						} // <--- compute position estimation by odometry on global coordinate

						// set position
						pos.x = pos_odmest[0][0];
						pos.y = pos_odmest[1][0];
						pos.theta += move_est.theta;

						optimizer->set_starting_value( starting, pos.x, pos.y, pos.theta );
					} // <--- add movement estimation
				}  // <--- 1. compute position estimation from odometry



				// ---> 2. set position estimation by odometry to optimization starting value
				optimizer->begin(starting);


				gnd::matrix::set_zero(&move_opt);
				{ // ---> 3. optimization iteration by matching laser scanner reading to map(likelihood field)
					double left_timer = 1.0;
					gnd::matrix::fixed<3,1> delta;
					gnd::matrix::fixed<2,1> reflect_prevent;

					// clear previous entered sensor reading
					gnd::matrix::set_zero(&reflect_prevent);

					// ---> scanning loop for sokuikiraw-data
					for(size_t i = 0; i < sokuikiraw.numPoints(); i++){
						// ---> entry laser scanner reflection
						gnd::matrix::fixed<4,1> reflect_csns, reflect_crbt;
						gnd::matrix::fixed<3,1> ws3x1;
						gnd::matrix::fixed<3,3> ws3x3;

						// if range data is null because of no reflection
						if( sokuikiraw[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
						// ignore error data
						else if( sokuikiraw[i].isError()) 	continue;
						else if( sokuikiraw[i].r < sokuikiraw_prop.distMin * 1.1)	continue;
						else if( sokuikiraw[i].r > sokuikiraw_prop.distMax * 0.9)	continue;

						{ // ---> compute laser scanner reading position on robot coordinate
							// set search position on sensor-coordinate
							reflect_csns[0][0] = sokuikiraw[i].r * ::cos( sokuikiraw[i].th );
							reflect_csns[1][0] = sokuikiraw[i].r * ::sin( sokuikiraw[i].th );
							reflect_csns[2][0] = 0;
							reflect_csns[3][0] = 1;


							// data decimation with distance threshold
							if( gnd_square(reflect_csns[0][0] - reflect_prevent[0][0]) + gnd_square(reflect_csns[1][0] - reflect_prevent[1][0]) < decimate_sqdist ){
								continue;
							}
							else {
								// update previous entered data
								gnd::matrix::copy(&reflect_prevent, &reflect_csns);
							}

							// convert from sensor coordinate to robot coordinate
							gnd::matrix::prod(&coordm_sns2rbt, &reflect_csns, &reflect_crbt);
						} // <--- compute laser scanner reading position on robot coordinate

						// data entry
						optimizer->entry( reflect_crbt[0][0] , reflect_crbt[1][0] );
						// <--- entry laser scanner reflection
					} // <--- scanning loop for sokuikiraw-data

					// zero reset likelihood
					lkl = 0;
					// zero reset optimization iteration counter
					opt_cnt = 0;
					gnd::matrix::set_zero(&move_opt);
					do{
						// store previous optimization position likelihood
						lkl_prev_opt = lkl;

						{ // ---> step iteration of optimization
							gnd::matrix::fixed<3,1> ws3x1;
							if( (ret = optimizer->iterate(&delta, &ws3x1, 0, &lkl)) < 0 ){
								break;
							}

							// get optimized position
							pos_opt.x = ws3x1[0][0];
							pos_opt.y = ws3x1[1][0];
							pos_opt.theta = ws3x1[2][0];
							// get movement by optimization
							gnd::matrix::add(&move_opt, &delta, &move_opt);
						} // <--- step iteration of optimization

						// loop counting
						opt_cnt++;
						// convergence test
//						timer_operate.clock(&left_timer);
					} while( !optimizer->converge_test() && left_timer > gnd_msec2time(30)); // <--- position optimization loop
				} // ---> 3. optimization iteration by matching laser scanner reading to map(likelihood field)



				// ---> 4. optimization error test and write position ssm-data
				// check --- 1st. function error, 2nd. distance, 3rd. orient difference
				if( ret >= 0 &&
						gnd_square( pos.x - pos_opt.x ) + gnd_square( pos.y - pos_opt.y ) < gnd_square( 1 ) &&
						::fabs( pos.theta - pos_opt.theta ) < gnd_deg2ang(10) ) {

					if( gnd_sign(running) > 0 ) {
						gnd::odometry::rsemap::counting(&emap, pos_opt.x, pos_opt.y, pos_opt.theta,
								running, (pos.x - pos_opt.x), (pos.y - pos_opt.y), (pos.theta - pos_opt.theta) );
					}
//					else {
//						gnd::odometry::rsemap::counting(&emap, pos_opt.x, pos_opt.y, pos_opt.theta + M_PI,
//								gnd_sign(running) * running, gnd_sign(running) * (pos_opt.x - pos.x), gnd_sign(running) * (pos_opt.y - pos.y), gnd_sign(running) * (pos_opt.theta - pos.theta) );
//					}

					pos.x = pos_opt.x;
					pos.y = pos_opt.y;
					pos.theta = pos_opt.theta;


                    ::fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                    ssmlog_odm.time() - init_time,
                                    ssmlog_odm.data().x, ssmlog_odm.data().y, ssmlog_odm.data().theta,
                                    ssmlog_odm.data().v, ssmlog_odm.data().w,
                                    pos.x, pos.y, pos.theta,
                                    pos.v, pos.w );

					running = 0;
				}
				else {
//					ssmlog_pos.write( ssmlog_sokuikiraw.time() );
				} // <--- 4. optimization error test and write position ssm-data


				{ // ---> 5. update robot position coordinate and odometory position coordinate
					gnd::matrix::fixed<4,4> coordm;

					// odometry coordinate
					gnd::matrix::coordinate_converter(&coordm,
							ssmlog_odm.data().x, ssmlog_odm.data().y, 0,
							::cos(ssmlog_odm.data().theta), ::sin(ssmlog_odm.data().theta), 0,
							 0, 0, 1);
					coordtree.set_coordinate(coordid_odm, &coordm);

					// robot position coordinate
					gnd::matrix::coordinate_converter(&coordm,
							pos.x, pos.y, 0,
							::cos( pos.theta ), ::sin( pos.theta ), 0,
							 0, 0, 1);
					coordtree.set_coordinate(coordid_rbt, &coordm);

					// get coordinate convert matrix
					coordtree.get_convert_matrix(coordid_sns, coordid_gl, &coordm_sns2gl);
				} // ---> 5. update robot position coordinate and odometory position coordinate



				{ // ---> 6. update map and view data
					// map update check, 1st. time, 2nd. position, 3rd. orient
					map_update = ssmlog_sokuikiraw.time() - time_premap > param.rest_cycle.value ||
							gnd_square( pos.x - pos_premap.x) + gnd_square( pos.y - pos_premap.y) > sqdist_mapup ||
							::fabs( pos.theta - pos_premap.theta ) > param.rest_orient.value;

					if( map_update && !param.slam.value ){ // ---> clear
						gnd::opsm::clear_counting_map(&cnt_map);
					} // <--- clear

					{// ---> scanning loop for sokuikiraw-data
						gnd::matrix::fixed<4,1> reflect_csns;
						gnd::matrix::fixed<4,1> reflect_cgl;
						gnd::matrix::fixed<2,1> reflect_prevent;

						OPSMPosTrack::Viewer::scan_cur.wait();
						{ // set view point data
							OPSMPosTrack::Viewer::scan_2prev.wait();
							OPSMPosTrack::Viewer::scan_prev.wait();
							// shift prev data
							OPSMPosTrack::Viewer::scan_2prev.var.clear();
							if( OPSMPosTrack::Viewer::scan_prev.var.begin() )
								OPSMPosTrack::Viewer::scan_2prev.var.copy(OPSMPosTrack::Viewer::scan_prev.var.begin(), OPSMPosTrack::Viewer::scan_prev.var.size() );
							OPSMPosTrack::Viewer::scan_prev.var.clear();
							if( OPSMPosTrack::Viewer::scan_cur.var.begin() )
								OPSMPosTrack::Viewer::scan_prev.var.copy(OPSMPosTrack::Viewer::scan_cur.var.begin(), OPSMPosTrack::Viewer::scan_cur.var.size() );
							OPSMPosTrack::Viewer::scan_prev.post();
							OPSMPosTrack::Viewer::scan_2prev.post();
						}
						OPSMPosTrack::Viewer::scan_cur.var.clear();
						// ---> scanning loop of laser scanner reading
						for(size_t i = 0; i < sokuikiraw.numPoints(); i++){
							gnd::matrix::fixed<3,1> ws3x1;
							gnd::matrix::fixed<3,3> ws3x3;
							gnd::gl::point p;

							// if range data is null because of no reflection
							if( sokuikiraw[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
							// ignore error data
							else if( sokuikiraw[i].isError()) 	continue;
							else if( sokuikiraw[i].r < sokuikiraw_prop.distMin)	continue;
							else if( sokuikiraw[i].r > sokuikiraw_prop.distMax)	continue;

							{ // ---> compute laser scanner reading position on global coordinate
								// set search position on sensor-coordinate
								gnd::matrix::set(&reflect_csns, 0, 0, sokuikiraw[i].r * ::cos( sokuikiraw[i].th ));
								gnd::matrix::set(&reflect_csns, 1, 0, sokuikiraw[i].r * ::sin( sokuikiraw[i].th ) );
								gnd::matrix::set(&reflect_csns, 2, 0, 0);
								gnd::matrix::set(&reflect_csns, 3, 0, 1);

								// data decimation with distance threshold
								if( gnd_square(reflect_csns[0][0] - reflect_prevent[0][0]) + gnd_square(reflect_csns[1][0] - reflect_prevent[1][0]) < decimate_sqdist ){
									continue;
								}
								else {
									// update previous entered data
									gnd::matrix::copy(&reflect_prevent, &reflect_csns);
								}

								// convert from sensor coordinate to global coordinate
								gnd::matrix::prod(&coordm_sns2gl, &reflect_csns, &reflect_cgl);
							} // <--- compute laser scanner reading position on global coordinate

							// ---> enter laser scanner reading to map
							if( map_update ) {
								if( param.slam.value ){
									gnd::opsm::update_map(&cnt_map, &op_map, reflect_cgl[0][0], reflect_cgl[1][0], gnd_m2dist(10), gnd_mm2dist(1), 0.5);
								}
								else {
									gnd::opsm::counting_map(&cnt_map, reflect_cgl[0][0], reflect_cgl[1][0]);
								}
								// update
								time_premap = ssmlog_sokuikiraw.time();
								pos_premap = pos;
							} // <--- enter laser scanner reading to map

							// enter viewer data
							p.x = reflect_cgl[0][0];
							p.y = reflect_cgl[1][0];
							p.z = 0.0;
							OPSMPosTrack::Viewer::scan_cur.var.push_back(&p);
						} // <--- scanning loop of laser scanner reading
						OPSMPosTrack::Viewer::scan_cur.post();

					} // <--- scanning loop for sokuikiraw-data


					// ---> update map
					if( map_update) {
						if( param.slam.value ){
						}
						else {
							if( param.ndt.value ){
								if( gnd::opsm::build_map(&op_map, &cnt_map, 10, 1.0e-3, 0) < 0 ){
									::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
								}
							}
							else{
								if( gnd::opsm::build_ndt_map(&op_map, &cnt_map, 1.0e-3) < 0 ){
									::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
								}
							}
						}
					} // <--- update map
				} // ---> 6. update map and view data
				odo_prevloop = ssmlog_odm.data();
				cnt++;
			} // <--- read ssm sokuikiraw

		} // <--- operation loop

	} // <--- operation



	{ // ---> finalization

		if(fp) ::fclose(fp);

		optimizer->delete_starting_value(&starting);
		delete optimizer;

		// slam
		if( param.slam.value ) {
			gnd::bmp32_t bmp;
			gnd::bmp8_t bmp8;

			gnd::opsm::build_map(&op_map, &cnt_map, gnd_m2dist(10), gnd_mm2dist(1), 5);

			{ // ---> write intermediate file
				::fprintf(stderr, " => write intermediate file\n");

				if( gnd::opsm::write_counting_map(&cnt_map, "map") ) {
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open\n");
				}
				else {
					::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save counting map data\n");
				}

			} // <--- write intermediate file

			// bmp file building
			::fprintf(stderr, " => bmp map building\n");
			gnd::opsm::build_bmp32(&bmp, &op_map, gnd_m2dist( 1.0 / 8) );

			{ // ---> file out
				{ // ---> bmp
					char fname[128];
					::fprintf(stderr, " => start map file out \n");

					::sprintf(fname, "%s.%s", "out", "bmp" );

					if( gnd::bmp::write32(fname, &bmp) < 0) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
					else {
						::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save map data into \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
				} // <--- bmp

				{ // ---> origin
					char fname[128];
					FILE *fp;
					double x, y;

					::sprintf(fname, "%s.%s", "out", "origin.txt" );

					fp = fopen(fname, "w");

					bmp.pget_origin(&x, &y);
					fprintf(fp, "%lf %lf\n", x, y);
					fclose(fp);
				} // --->  origin
			} // <--- fileout


			// bmp file building
			::fprintf(stderr, " => bmp map building\n");
			gnd::opsm::build_bmp8(&bmp8, &op_map, gnd_m2dist( 1.0 / 8) );

			{ // ---> file out
				{ // ---> bmp
					char fname[128];
					::fprintf(stderr, " => start map file out \n");

					::sprintf(fname, "%s.%s", "out8", "bmp" );

					if( gnd::bmp::write8(fname, &bmp8) < 0) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
					else {
						::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save map data into \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
				} // <--- bmp

				{ // ---> origin
					char fname[128];
					FILE *fp;
					double x, y;

					::sprintf(fname, "%s.%s", "out8", "origin.txt" );

					fp = fopen(fname, "w");

					bmp.pget_origin(&x, &y);
					fprintf(fp, "%lf %lf\n", x, y);
					fclose(fp);
				} // --->  origin
			} // <--- fileout

			// viwer free
			gnd::gl::finalize();

		}

		{ // ---> file out
			gnd::odometry::rsemap::pxl *p;
			double x, y;
			char fname[128];
			char mapfname[128];

			for( int i = 0; i < emap.angrsl; i++ ) {
				FILE *fp;
				::sprintf(fname, "map%d.dat", i);
				fp = ::fopen(fname, "w");
				::fprintf(fp, "# angular range %lf %lf\n", gnd::odometry::rsemap::orient(&emap, i), gnd::odometry::rsemap::orient_upper(&emap, i));

				::sprintf(mapfname, "map%d.rmap", i);
				// ---> row
				for( unsigned int r = 0; r < emap.p[i].row(); r++ ) {
					// ---> column
					for( unsigned int c = 0; c < emap.p[i].column(); c++ ) {
						p = emap.p[i].pointer(r, c);
						emap.p[i].pget_pos_core(r, c, &x, &y);
						::fprintf( fp, "%lf %lf, %lf %lf %lf, %lf\n", x, y,
								p->dist > 0.3 ? p->dx / p->dist : 0, p->dist > 0.3 ? p->dy / p->dist : 0, p->dist > 0.3 ? gnd_ang2deg(p->dtheta / p->dist) : 0,
								p->dist);

						emap.p[i].fwrite(mapfname);
					} // <--- column
				} // <--- row
				::fclose(fp);
			}

			gnd::odometry::rsemap::write("rse-map", &emap);
		} // <--- file out

		::fprintf(stderr, "\n");
		::fprintf(stderr, "Finish\n");

	} // <--- finalization

	return 0;
}





