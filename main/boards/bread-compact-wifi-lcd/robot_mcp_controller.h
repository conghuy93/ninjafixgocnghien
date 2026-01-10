#ifndef __ROBOT_MCP_CONTROLLER_H__
#define __ROBOT_MCP_CONTROLLER_H__

#include "mcp_server.h"
#include "robot_control.h"
#include <esp_log.h>

#define TAG_MCP "RobotMCP"

class RobotMcpController {
public:
    RobotMcpController() {
        auto& mcp_server = McpServer::GetInstance();
        
        // Tool 1: Move forward (tiến lên phía trước)
        // X=0, Y=100 trong 3 giây
        mcp_server.AddTool("robot.move_forward", 
            "[Otto Ninja Robot] Di chuyển robot tiến về phía trước trong 3 giây. "
            "Robot Ninja có 2 chế độ: WALK (đi bộ bằng chân) và ROLL (lăn bánh xe). "
            "Lệnh này hoạt động ở cả 2 chế độ.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Move forward 3s - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                // Set joystick: X=0, Y=100
                state->j_x = 0;
                state->j_y = 100;
                
                // Wait for 3 seconds
                vTaskDelay(pdMS_TO_TICKS(3000));
                
                // Stop
                state->j_x = 0;
                state->j_y = 0;
                
                return std::string("Robot Ninja đã tiến về phía trước 3 giây ở chế độ ") + 
                       (mode == MODE_WALK ? "ĐI BỘ" : "LĂN");
            });
        
        // Tool 2: Move backward (lùi)
        // X=0, Y=-100 trong 3 giây
        mcp_server.AddTool("robot.move_backward", 
            "[Otto Ninja Robot] Di chuyển robot lùi về phía sau trong 3 giây. "
            "Robot Ninja có 2 chế độ: WALK (đi bộ bằng chân) và ROLL (lăn bánh xe). "
            "Lệnh này hoạt động ở cả 2 chế độ.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Move backward 3s - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                // Set joystick: X=0, Y=-100
                state->j_x = 0;
                state->j_y = -100;
                
                // Wait for 3 seconds
                vTaskDelay(pdMS_TO_TICKS(3000));
                
                // Stop
                state->j_x = 0;
                state->j_y = 0;
                
                return std::string("Robot Ninja đã lùi về phía sau 3 giây ở chế độ ") + 
                       (mode == MODE_WALK ? "ĐI BỘ" : "LĂN");
            });
        
        // Tool 3: Turn left (quay trái)
        // WALK mode: Nghiêng trái (giống bấm nút Web UI) + Đợi 1s + Rotate LF 0.5s
        // ROLL mode: X=-75, Y=-64
        mcp_server.AddTool("robot.turn_left", 
            "[Otto Ninja Robot] Quay robot sang trái. "
            "Chế độ WALK: nghiêng trái (giống nút Web UI), đợi 1 giây, rồi xoay chân trái 0.5 giây. "
            "Chế độ ROLL: sử dụng joystick để quay trái.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Turn left - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                if (mode == MODE_WALK) {
                    // WALK mode: Call ninja_tilt_left() exactly like Web UI button
                    ninja_tilt_left();  // This sets manual_mode = true internally
                    
                    // Wait 1 second after tilting (like user requested)
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // Rotate Left Foot for 0.5s
                    calibration_t* cal = get_calibration();
                    int lf_angle = cal->lf_neutral + cal->lffwrs;
                    servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    servo_direct_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
                    
                    // Return to home
                    state->manual_mode = false;
                    go_home();
                    
                    return std::string("Robot Ninja quay trái xong (nghiêng + xoay LF)");
                } else {
                    // ROLL mode: X=-75, Y=-64
                    state->j_x = -75;
                    state->j_y = -64;
                    
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    state->j_x = 0;
                    state->j_y = 0;
                    
                    return std::string("Robot Ninja quay trái xong (chế độ lăn)");
                }
            });
        
        // Tool 4: Turn right (quay phải)
        // WALK mode: Nghiêng phải (giống bấm nút Web UI) + Đợi 1s + Rotate RF 0.5s
        // ROLL mode: X=51, Y=-81
        mcp_server.AddTool("robot.turn_right", 
            "[Otto Ninja Robot] Quay robot sang phải. "
            "Chế độ WALK: nghiêng phải (giống nút Web UI), đợi 1 giây, rồi xoay chân phải 0.5 giây. "
            "Chế độ ROLL: sử dụng joystick để quay phải.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Turn right - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                if (mode == MODE_WALK) {
                    // WALK mode: Call ninja_tilt_right() exactly like Web UI button
                    ninja_tilt_right();  // This sets manual_mode = true internally
                    
                    // Wait 1 second after tilting (like user requested)
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // Rotate Right Foot for 0.5s
                    calibration_t* cal = get_calibration();
                    int rf_angle = cal->rf_neutral - cal->rffwrs;
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
                    
                    // Return to home
                    state->manual_mode = false;
                    go_home();
                    
                    return std::string("Robot Ninja quay phải xong (nghiêng + xoay RF)");
                } else {
                    // ROLL mode: X=51, Y=-81
                    state->j_x = 51;
                    state->j_y = -81;
                    
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    state->j_x = 0;
                    state->j_y = 0;
                    
                    return std::string("Robot Ninja quay phải xong (chế độ lăn)");
                }
            });
        
        // Tool 5: Get robot mode (kiểm tra mode hiện tại)
        mcp_server.AddTool("robot.get_mode", 
            "[Otto Ninja Robot] Kiểm tra chế độ hiện tại của robot Ninja. "
            "Robot có 2 chế độ: WALK (đi bộ bằng chân) và ROLL (lăn bánh xe).",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                robot_mode_t mode = get_robot_mode();
                if (mode == MODE_WALK) {
                    return std::string("{\"mode\": \"walk\", \"description\": \"Chế độ ĐI BỘ - robot Ninja đi bằng 2 chân\"}");
                } else {
                    return std::string("{\"mode\": \"roll\", \"description\": \"Chế độ LĂN - robot Ninja lăn bằng bánh xe\"}");
                }
            });
        
        // Tool 6: Set mode to walk
        mcp_server.AddTool("robot.set_walk_mode", 
            "[Otto Ninja Robot] Chuyển robot sang chế độ ĐI BỘ (WALK). "
            "Robot sẽ đứng bằng 2 chân và di chuyển bằng cách bước đi.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "Setting walk mode");
                ninja_set_walk();
                return std::string("Robot Ninja đã chuyển sang chế độ ĐI BỘ");
            });
        
        // Tool 7: Set mode to roll
        mcp_server.AddTool("robot.set_roll_mode", 
            "[Otto Ninja Robot] Chuyển robot sang chế độ LĂN (ROLL). "
            "Robot sẽ hạ thấp xuống và di chuyển bằng bánh xe.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "Setting roll mode");
                ninja_set_roll();
                return std::string("Robot Ninja đã chuyển sang chế độ LĂN");
            });
        
        // Tool 8: Go home position
        mcp_server.AddTool("robot.go_home", 
            "[Otto Ninja Robot] Đưa robot về vị trí HOME (đứng thẳng trung tính). "
            "Robot sẽ trở về tư thế đứng chuẩn.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "Going home");
                go_home();
                return std::string("Robot Ninja đã về vị trí HOME");
            });
        
        ESP_LOGI(TAG_MCP, "Robot MCP tools registered successfully (8 tools)");
    }
};

#endif // __ROBOT_MCP_CONTROLLER_H__
