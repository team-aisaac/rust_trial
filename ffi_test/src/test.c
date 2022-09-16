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
	fprintf(stdout, "%d %d\n", *targetX, *targetY);
	*targetX = 80;

	fprintf(stdout, "----------------\n");

	fprintf(stdout, "%d %d\n", *vx_out, *ay_out);
	*vx_out = 101;
	*ay_out = 101;
	fprintf(stdout, "%d %d\n", *vx_out, *ay_out);

	fprintf(stdout, "----------------\n");
}
