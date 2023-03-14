/**
 * @file tracker.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-02
 * 
 */

#include "armor_tracker/tracker_kalman.h"


ArmorTrackerKalman::ArmorTrackerKalman(const KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
        int lost_threshold)
    : tracker_state(LOST),
    tracking_id(0),
    kf_matrices_(kf_matrices),
    tracking_velocity_(Eigen::Vector3d::Zero()),
    max_lost_distance(max_match_distance),
    matched_count_threshold(tracking_threshold),
    lost_count_threshold(lost_threshold)
{
}

void ArmorTrackerKalman::init(std::vector<Armor> & armors, double timestmp){
    
    if (armors.empty()) 
        return;

    double max_area = 0;
    auto chosen_armor = armors[0];
    for (auto armor : armors){
        if (armor.area > max_area){
            chosen_armor = armor;
            max_area = armor.area;
        }
    }

    // KF init
    kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
    Eigen::VectorXd init_state(6);
    const auto position = chosen_armor.center3d_world;
    init_state << position.x(), position.y(), position.z(), 0, 0, 0;
    kf_->init(init_state);

    tracking_id = chosen_armor.id;
    tracker_state = DETECTING;
}

/**
 * @brief 进行跟踪装甲板，更新建议击打装甲板
 * @param dt_ 
 */
void ArmorTrackerKalman::update(const std::vector<Armor> & armors, double dt_){

    // KF predict
    kf_matrices_.F(0, 3) = kf_matrices_.F(1, 4) = kf_matrices_.F(2, 5) = dt_;
    Eigen::VectorXd kf_prediction = kf_->predict(kf_matrices_.F);

    bool matched = false;
    // Use KF prediction as default target state if no matched armor is found
    target_state = kf_prediction;

    if (!armors.empty()) {
        Armor matched_armor;
        double min_position_diff = DBL_MAX;
        for (const auto & armor : armors) {
            Eigen::Vector3d position_vec = armor.center3d_world;
            Eigen::Vector3d predicted_position = kf_prediction.head(3);
            // Difference of the current armor position and tracked armor's predicted position
            double position_diff = (predicted_position - position_vec).norm();
            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                matched_armor = armor;
            }
        }
        suggest_armor = matched_armor;

        if (min_position_diff < max_lost_distance) {
            // Matching armor found
            matched = true;
            target_state = kf_->update(matched_armor.center3d_world);
        } else {
            // Check if there is same id armor in current frame
            for (const auto & armor : armors) {
                if (armor.id == tracking_id) {
                    matched = true;
                    // Reset KF
                    kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
                    Eigen::VectorXd init_state(6);
                    // Set init state with current armor position and tracking velocity before
                    init_state << armor.center3d_world, tracking_velocity_;
                    kf_->init(init_state);
                    target_state = init_state;
                    suggest_armor = armor;
                    break;
                }
            }
        }
    }

    // Save tracking target velocity
    tracking_velocity_ = target_state.tail(3);

    // Tracking state machine
    if (tracker_state == DETECTING) {
        // DETECTING
        if (matched) {
        matched_count++;
        if (matched_count > matched_count_threshold) {
            matched_count = 0;
            tracker_state = TRACKING;
        }
        } else {
            matched_count = 0;
            tracker_state = LOST;
        }

    } else if (tracker_state == TRACKING) {
        // TRACKING
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count++;
        }

    } else if (tracker_state == TEMP_LOST) {
        // LOST
        if (!matched) {
        lost_count++;
        if (lost_count > lost_count_threshold) {
            lost_count = 0;
            tracker_state = LOST;
        }
        } else {
            tracker_state = TRACKING;
            lost_count = 0;
        }
    }
}