#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

/* 우산 헤더 - 예전 robot_runner.h 자리.
 * 8,375줄 단일 헤더를 아래 모듈로 쪼갠 뒤, 기존 include 한 줄을
 * 그대로 쓰게 하려고 남겨둔 파일이다. 새 코드는 필요한 모듈만
 * 직접 include 하는 쪽이 낫다. */

#include "robot_config.h"
#include "robot_odom.h"
#include "robot_scan.h"
#include "robot_localize.h"
#include "robot_motion.h"
#include "robot_rotrate.h"
#include "robot_sense.h"
#include "robot_escape.h"
#include "robot_relocalize.h"
#include "robot_slot_geom.h"
#include "robot_pose_gate.h"
#include "robot_slot_align.h"
#include "robot_slot_drive.h"
#include "robot_map.h"
#include "robot_navigate.h"
#include "robot_shuttle.h"

#include "corner_escape.h"

#endif /* ROBOT_RUNNER_H */
