#include "fm_pdr.h"

int fm_pdr_init(char* config_path, void* train_data, PDRTrajectory** trajectories) {return 0;}
int fm_pdr_predict(PDRPoint start_point, void* process_data, PDRTrajectory** trajectories) {return 0;}
int fm_pdr_calibration(void *anchor_point, PDRTrajectory** trajectories) {return 0;}
void fm_pdr_uninit() {}