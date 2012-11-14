/*
 * opsm-position-tracker-offline-cui.hpp
 *
 *  Created on: 2012/11/14
 *      Author: tyamada
 */

#ifndef OPSM_POSITION_TRACKER_OFFLINE_CUI_HPP_
#define OPSM_POSITION_TRACKER_OFFLINE_CUI_HPP_

#include "gnd-cui.hpp"

namespace ObservationProbabilityScanMatching {
	namespace PositionTracker {

		static const gnd::cui_command cui_cmd[] = {
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
	}
}


#endif /* OPSM_POSITION_TRACKER_OFFLINE_CUI_HPP_ */
