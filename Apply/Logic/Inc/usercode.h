#ifndef	__USERCODE_H_
#define	__USERCODE_H_

void UserLogic_Code(void);
void thruster_start_open(void);


void rotate_180_adjust(int left_offset,int right_offset);
void forward_adjust(int left_offset,int right_offset);

void jy901_yaw_anti_forward_open(void);
void jy901_yaw_anti_rotate_open(void);

void jy901_yaw_anti_rotation_cancel(void);



#endif
