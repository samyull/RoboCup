// // #include "map.h"
// #include <Arduino.h>

// static float prev_heading = 0;

// static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

// //filtered sensors with offsets
// static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
//     {TOF1, {0, {-100, 136}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))},
//     {TOF2, {0, {100, 136}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))}
// };

// //non filtered sensors with offset
// static location_sensor_t opticalFlow_sensor = {NON_FILTERED, {FLOW_THETA_OFFSET, {FLOW_X_OFFSET, FLOW_Y_OFFSET}}, PoseMatrix::buildFrame(&(opticalFlow_sensor.offset))};

// //define world origin
// static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

// static PoseMatrix robot_worldFrame = PoseMatrix::buildFrame(&worldOrigin);

// /*******************************************************************************************************************************************************************************************
//  * converting to grid
//  */

// static void logCell(int gx, int gy, int8_t value) {
//     Serial.print("C, "); Serial.print(gx); Serial.print(", "); Serial.print(gy);
//     Serial.print(", "); Serial.println(value);
// }

// static void logPose(float theta, float x, float y) {
//     Serial.print("P, "); Serial.print(theta, 4); Serial.print(", ");
//     Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2);
// }

// bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
//     //converts world coords to map indeces, then returns true is point is on the map
//     grid_x = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
//     grid_y = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;

//     return(grid_x >= 0 && grid_x < GRID_WIDTH && grid_y >= 0 && grid_y < GRID_HEIGHT);
// }

// void writeToMap(float world_x, float world_y, int8_t value) {
//     //write a value to a cell on the map
//     int gx, gy;

//     if (worldToGrid(world_x, world_y, gx, gy)) {
//         mapData[gy][gx] = value;
//         logCell(gx, gy, value);
//     }
// }

// static void rayTraceFree(int x0, int y0, int x1, int y1) {
//     int dx = abs(x1- x0), sx = x0 < x1 ? 1 : -1;
//     int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
//     int error = dx + dy;
//     int x = x0, y = y0;

//     while (!(x == x1 && y == y1)) {
//         int e2 = 2 * error;
//         if (e2 >= dy) {error += dy; x += sx;}
//         if (e2 <= dx) {error += dx; y += sy;}
//         if (x == x1 && y == y1) break;

//         if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
//             if (mapData[y][x] != static_cast<int8_t>(WEIGHT)) {
//                 mapData[y][x] = static_cast<int8_t>(VALID_POS);
//                 logCell(x, y, static_cast<int8_t>(VALID_POS));
//             }
//         }
//     }
// }

// /*****************************************************************************************************************************************************************************************
//  * projecting pose
//  */

// void updatePose(pos_vector_t* Pose) {
//     //overwrite last pos
//     writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(VALID_POS));

//     //get new POS
//     PoseMatrix delta = PoseMatrix::buildFrame(Pose);
//     robot_worldFrame = robot_worldFrame.transformFrame(delta);

//     //write current POS
//     writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(MY_POS));
// }

// void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight) {
//     //transform sensor frame into world coords
//     PoseMatrix sensor_worldFrame = robot_worldFrame.transformFrame(sensor_robotFrame);

//     pos_t worldCoord;
//     sensor_worldFrame.transformCoords(sensorVector, worldCoord);

//     if (!isWeight) {
//         int xr, yr, xw, yw;
//         worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), xr, yr);
//         worldToGrid(worldCoord.x, worldCoord.y, xw, yw);
//         rayTraceFree(xr, yr, xw, yw);
//     }

//     writeToMap(worldCoord.x, worldCoord.y, isWeight? static_cast<int8_t>(WEIGHT) : static_cast<int8_t>(INVALID_CELL));
// }

// pos_t projectOpticalFlow(const location_sensor_t& opticalFlow_sensor, pos_t* sensorVector, float theta) {
//     //project the reading from the optical flow sensor into robot coords
//     pos_t robotCoord;
//     opticalFlow_sensor.sensor_robotFrame.rotateVector(sensorVector, robotCoord);

//     //correct for arc rotation caused by sensor offset
//     float x = opticalFlow_sensor.offset.pos.x;
//     float y = opticalFlow_sensor.offset.pos.y;
//     theta = theta * D_TO_R;
//     float c = std::cos(theta), s = std::sin(theta);
//     float corr_x = (c - 1.0f) * x - s * y;
//     float corr_y = s * x + (c - 1.0f) * y;

//     robotCoord.x -= corr_x;
//     robotCoord.y -= corr_y;

//     return robotCoord;
// }

// /*************************************************************************************************************************************************************************************************
//  * overall func
//  */

// void poseTask() {
//     //get how far robot have moved since the last time we did this
//     //heading from IMU, dx and dy from encoders or IR camera or smth
//     float heading = getHeading();
//     float differential_heading = prev_heading - heading;
//     prev_heading = heading;

//     //put x and y reading from optical flow sensor into a displacement vector
//     pos_t displacement;
//     read_opticalFlow(displacement.y, displacement.x);

//     pos_vector_t robot_displacement = {
//         differential_heading,
//         projectOpticalFlow(opticalFlow_sensor, &displacement, differential_heading)
//     };

//     updatePose(&robot_displacement);

//     //find objects with the TOFs
//     for (int i = 0; i < BUFFERCOUNT; i++) {
//         pos_t sensorVector = {0, (float)getOutput(filtered_sensors[i].ID)};

//         bool seenWeight = 0;
//         if (filtered_sensors[i].ID == (TOF1 || TOF2)) seenWeight = determineWeight(filtered_sensors[i].ID);

//         projectSensorHit(filtered_sensors[i].sensor_robotFrame, &sensorVector, seenWeight);
//     }

//     logPose(robot_worldFrame.theta(), robot_worldFrame.x(), robot_worldFrame.y());
// }