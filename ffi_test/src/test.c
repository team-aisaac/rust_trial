#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void hello_world() {
	fprintf(stdout, "hello from C\n");
}

void execDWA(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t *targetX, int32_t *targetY, int32_t *targetTheta, int32_t *middle_targetX, int32_t *middle_targetY, int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[], int32_t ObstacleVX[], int32_t ObstacleVY[], bool prohibited_zone_ignore, bool *middle_target_flag, bool *is_enable, bool *path_enable, bool *prohibited_zone_start, int32_t *vx_out, int32_t *vy_out, int32_t *omega_out, int32_t *ax_out, int32_t *ay_out) {
	fprintf(stdout, "----------------\n");
	fprintf(stdout, "%d %d\n", x, y);
	fprintf(stdout, "%d %d\n", Vx, Vy);
	fprintf(stdout, "%d %d\n", theta, omega);
	
	fprintf(stdout, "%d %d %d %d %d\n", numOfObstacle, ObstacleX[0], ObstacleY[0], ObstacleVX[0], ObstacleVY[0]);
	fprintf(stdout, "%d %d %d %d %d\n", numOfObstacle, ObstacleX[1], ObstacleY[1], ObstacleVX[1], ObstacleVY[1]);

	fprintf(stdout, "----------------\n");

	fprintf(stdout, "Update values\n");
	*targetX = 100;
	*targetY = 101;
	*targetTheta = 102;
	*middle_targetX = 103;
	*middle_targetY = 104;
	*middle_target_flag = true;
	*is_enable = true;
	*path_enable = true;
	*prohibited_zone_start = true;
	*vx_out = 105;
	*vy_out = 106;
	*omega_out = 107;
	*ax_out = 108;
	*ay_out = 109;

	fprintf(stdout, "----------------\n");
}
