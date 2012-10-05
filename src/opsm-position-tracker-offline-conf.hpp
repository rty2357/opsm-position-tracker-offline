/*
 * observation-probability-position-tracker-offline-conf.hpp
 *
 *  Created on: 2011/12/22
 *      Author: tyamada
 */

#ifndef OBSERVATION_PROBABILITY_POSITION_TRACKER_OFFLINE_CONF_HPP_
#define OBSERVATION_PROBABILITY_POSITION_TRACKER_OFFLINE_CONF_HPP_

#include <ssmtype/spur-odometry.h>
#include "ssm-laser.hpp"

#include "gnd-observation-probability.hpp"
#include "gnd-configuration.hpp"
#include "gnd-lib-error.h"

#ifndef OPSMPosTrack
#define OPSMPosTrack ObservationProbabilityScanMatching::PositionTracker
#endif

#define __OPTIMIZER_NEWTON__		"newton"
#define __OPTIMIZER_QMC__	 		"qmc"
#define __OPTIMIZER_QMC2NEWTON__	"qmc2newton"

namespace ObservationProbabilityScanMatching {
	namespace PositionTracker {
		static const char proc_name[] = "opsm-position-tracker";

		// map-file
		static const gnd::Conf::parameter_array<char, 512> ConfIni_MapDir = {
				"map-dir",
				"",		// map file directory
		};
		// ssm-name
		static const gnd::Conf::parameter_array<char, 512> ConfIni_OdometryLogName = {
				"odm-log-name",
				"",
		};

		// laser scanner ssm-name
		static const gnd::Conf::parameter_array<char, 512> ConfIni_LaserScannerLogName = {
				"laser-scanner-log-name",
				"",
		};
		// decimate threshold
		static const gnd::Conf::parameter<double> ConfIni_Decimate = {
				"decimate",
				gnd_m2dist( 0.08 ),	// [m]
		};

		// cycle
		static const gnd::Conf::parameter<double> ConfIni_Cycle = {
				"cycle",
				gnd_sec2time(0.2),	// [s]
		};

		// rest-cycle
		static const gnd::Conf::parameter<double> ConfIni_RestCycle = {
				"rest-cycle",
				gnd_sec2time(10.0),	// [s]
		};
		// rest-threshold-distance
		static const gnd::Conf::parameter<double> ConfIni_RestDist = {
				"rest-threshold-distance",
				gnd_m2dist(0.05),	// [m]
		};
		// rest-threshold-orientation
		static const gnd::Conf::parameter<double> ConfIni_RestOrient = {
				"rest-threshold-orientation",
				2.5,	// [deg]
		};

		// rest-threshold-orientation
		static const gnd::Conf::parameter<bool> ConfIni_SLAM = {
				"slam",
				false,
		};
		// optimizer

		static const char OptNewton[]		= __OPTIMIZER_NEWTON__;
		static const char OptQMC[]			= __OPTIMIZER_QMC__;
		static const char OptQMC2Newton[]	= __OPTIMIZER_QMC2NEWTON__;
		static const gnd::Conf::parameter_array<char, 512> ConfIni_Optimizer = {
				"optimizer",
				__OPTIMIZER_NEWTON__,		// map file directory
		};
		// converge dist
		static const gnd::Conf::parameter<double> ConfIni_ConvergeDist = {
				"converge-distance",
				gnd_m2dist(0.01),	// [m]
		};
		// converge orient
		static const gnd::Conf::parameter<double> ConfIni_ConvergeOrient = {
				"converge-orient",
				0.5,	// [deg]
		};

		// number of scan data for first map building
		static const gnd::Conf::parameter<int> ConfIni_IniMapCnt = {
				"ini-map-cnt",
				1,	// [deg]
		};

		// rest-threshold-orientation
		static const gnd::Conf::parameter<bool> ConfIni_NDT = {
				"ndt",
				false,
		};

		// number of scan data for first map building
		static const gnd::Conf::parameter<bool> ConfIni_DebugViewer = {
				"debug-viewer",
				true,
		};

		// number of scan data for first map building
		static const gnd::Conf::parameter<bool> ConfIni_DebugShowMode = {
				"debug-show-mode",
				true,
		};



		// number of scan data for first map building
		static const gnd::Conf::parameter<double> ConfIni_PosGridSize = {
				"road-map-pos-grid",
				0.5,	// [m]
		};

		// number of scan data for first map building
		static const gnd::Conf::parameter<int> ConfIni_AngReslution = {
				"road-map-ang-rsl",
				8,	// [m]
		};


		/**
		 * \brief particle localizer configure
		 */
		struct proc_configuration {
			proc_configuration();

			gnd::Conf::parameter_array<char, 512>	mapdir;				///< map data directory
			gnd::Conf::parameter_array<char, 512>	odm_logname;		///< odometry position estimation ssm-name
			gnd::Conf::parameter_array<char, 512>	ls_logname;			///< laser scanner data ssm-name
			gnd::Conf::parameter<double>			decimate;			///< laser scanner data decimate parameter [m]
			gnd::Conf::parameter<double>			cycle;				///< operation cycle
			gnd::Conf::parameter<double>			rest_cycle;			///< resting mode cycle
			gnd::Conf::parameter<double>			rest_dist;			///< resting threshold (position distance) [m]
			gnd::Conf::parameter<double>			rest_orient;		///< resting threshold (position orientation) [deg]
			gnd::Conf::parameter<bool>				slam;				///< slam mode flag
			gnd::Conf::parameter_array<char, 512>	optimizer;			///< kind of optimizer
			gnd::Conf::parameter<double>			converge_dist;		///< convergence test threshold (position distance) [m]
			gnd::Conf::parameter<double>			converge_orient;	///< convergence test threshold (position orientation) [deg]
			gnd::Conf::parameter<int>				ini_map_cnt;		///< number of scan data for first map building
			gnd::Conf::parameter<bool>				ndt;				///< ndt mode
			gnd::Conf::parameter<bool>				debug_viewer;		///< debug viewer
			gnd::Conf::parameter<bool>				debug_show;			///< debug show mode

			gnd::Conf::parameter<double>			pos_gridsize;		///< road map position size
			gnd::Conf::parameter<int>				ang_rsl;			///< road map angular resolution
		};
		typedef struct proc_configuration configure_parameters;

		/**
		 * @brief initialize configure to default parameter
		 */
		int configure_initialize(proc_configuration *conf);

		/**
		 * @brief analyze configure file
		 */
		int analyze_configure(gnd::Conf::Configuration *fconf, proc_configuration *confp);

		/**
		 * @brief constructor
		 */
		inline
		proc_configuration::proc_configuration(){
			configure_initialize(this);
		}

		/**
		 * @brief initialize configure
		 */
		inline
		int configure_initialize(proc_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->mapdir,				&ConfIni_MapDir,				sizeof(ConfIni_MapDir) );
			::memcpy(&conf->odm_logname,		&ConfIni_OdometryLogName,		sizeof(ConfIni_OdometryLogName) );
			::memcpy(&conf->ls_logname,			&ConfIni_LaserScannerLogName,	sizeof(ConfIni_LaserScannerLogName) );
			::memcpy(&conf->decimate,			&ConfIni_Decimate,				sizeof(ConfIni_Decimate) );
			::memcpy(&conf->cycle,				&ConfIni_Cycle,					sizeof(ConfIni_Cycle) );
			::memcpy(&conf->rest_cycle,			&ConfIni_RestCycle,				sizeof(ConfIni_RestCycle) );
			::memcpy(&conf->rest_dist,			&ConfIni_RestDist,				sizeof(ConfIni_RestDist) );
			::memcpy(&conf->rest_orient,		&ConfIni_RestOrient,			sizeof(ConfIni_RestOrient) );
			::memcpy(&conf->slam,				&ConfIni_SLAM,					sizeof(ConfIni_SLAM) );
			::memcpy(&conf->optimizer,			&ConfIni_Optimizer,				sizeof(ConfIni_Optimizer) );
			::memcpy(&conf->converge_dist,		&ConfIni_ConvergeDist,			sizeof(ConfIni_ConvergeDist) );
			::memcpy(&conf->converge_orient,	&ConfIni_ConvergeOrient,		sizeof(ConfIni_ConvergeOrient) );
			::memcpy(&conf->ini_map_cnt,		&ConfIni_IniMapCnt,				sizeof(ConfIni_IniMapCnt) );
			::memcpy(&conf->ndt,				&ConfIni_NDT,					sizeof(ConfIni_NDT) );
			::memcpy(&conf->debug_viewer,		&ConfIni_DebugViewer,			sizeof(ConfIni_DebugViewer) );
			::memcpy(&conf->debug_show,			&ConfIni_DebugShowMode,			sizeof(ConfIni_DebugShowMode) );

			::memcpy(&conf->pos_gridsize,		&ConfIni_PosGridSize,			sizeof(ConfIni_PosGridSize) );
			::memcpy(&conf->ang_rsl,			&ConfIni_AngReslution,			sizeof(ConfIni_AngReslution) );
			return 0;
		}

		/**
		 * @brief analyze
		 */
		inline
		int get_config_param(gnd::Conf::Configuration *conf, proc_configuration *confp)
		{
			gnd_assert(!conf, -1, "invalid null pointer");
			gnd_assert(!confp, -1, "invalid null pointer");

			gnd::Conf::get_parameter( conf, &confp->mapdir );
			gnd::Conf::get_parameter( conf, &confp->odm_logname );
			gnd::Conf::get_parameter( conf, &confp->ls_logname );
			gnd::Conf::get_parameter( conf, &confp->decimate );
			gnd::Conf::get_parameter( conf, &confp->cycle );
			gnd::Conf::get_parameter( conf, &confp->rest_cycle );
			gnd::Conf::get_parameter( conf, &confp->rest_dist );
			gnd::Conf::get_parameter( conf, &confp->rest_orient );
			gnd::Conf::get_parameter( conf, &confp->slam );
			gnd::Conf::get_parameter( conf, &confp->optimizer );
			gnd::Conf::get_parameter( conf, &confp->converge_dist );
			gnd::Conf::get_parameter( conf, &confp->converge_orient );
			gnd::Conf::get_parameter( conf, &confp->ini_map_cnt );
			gnd::Conf::get_parameter( conf, &confp->ndt );
			gnd::Conf::get_parameter( conf, &confp->debug_viewer );
			gnd::Conf::get_parameter( conf, &confp->debug_show );
			gnd::Conf::get_parameter( conf, &confp->pos_gridsize );
			gnd::Conf::get_parameter( conf, &confp->ang_rsl );

			confp->rest_orient.value = gnd_deg2rad(confp->rest_orient.value);
			return 0;
		}

		/**
		 * @brief file out  configure file
		 */
		inline
		int write_configuration( const char* fname, proc_configuration *confp ){

			gnd_assert(!fname, -1, "invalid null pointer");
			gnd_assert(!confp, -1, "invalid null pointer");

			{ // ---> operation
				gnd::Conf::FileStream conf;
				gnd::Conf::set_parameter(&conf, &confp->mapdir);
				gnd::Conf::set_parameter(&conf, &confp->odm_logname);
				gnd::Conf::set_parameter(&conf, &confp->ls_logname);
				gnd::Conf::set_parameter(&conf, &confp->decimate);
				gnd::Conf::set_parameter(&conf, &confp->cycle);
				gnd::Conf::set_parameter(&conf, &confp->rest_cycle);
				gnd::Conf::set_parameter(&conf, &confp->rest_dist);
				gnd::Conf::set_parameter(&conf, &confp->rest_orient);
				gnd::Conf::set_parameter(&conf, &confp->slam);
				gnd::Conf::set_parameter(&conf, &confp->optimizer);
				gnd::Conf::set_parameter(&conf, &confp->converge_dist);
				gnd::Conf::set_parameter(&conf, &confp->converge_orient);
				gnd::Conf::set_parameter(&conf, &confp->ini_map_cnt );
				gnd::Conf::set_parameter(&conf, &confp->ndt);
				gnd::Conf::set_parameter(&conf, &confp->debug_viewer );
				gnd::Conf::set_parameter(&conf, &confp->debug_show );

				gnd::Conf::set_parameter(&conf, &confp->pos_gridsize );
				gnd::Conf::set_parameter(&conf, &confp->ang_rsl );

				return conf.write(fname);
			} // <--- operation
		}

	} // namespace PositionTracker
}; // namespace ObservationProbabilityScanMatching

#undef __OPTIMIZER_NEWTON__
#undef __OPTIMIZER_QMC__
#undef __OPTIMIZER_QMC2NEWTON__

#endif /* OBSERVATION_PROBABILITY_POSITION_TRACKER_CONF_HPP_ */
