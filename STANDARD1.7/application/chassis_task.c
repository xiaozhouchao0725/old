/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
int anglesr;
#include "referee.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
//�����˶�����
chassis_move_t chassis_move;
//��ǿ���
int8_t QA=0;
//�������õ����趨������Ϣ��Ƶ��
int8_t feipo = 0;
//int16_t set = 100;
int8_t FCHO = 0;
extern gimbal_control_t gimbal_control;
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //���̳�ʼ��
    chassis_init(&chassis_move);
    //�жϵ��̵���Ƿ�����
//    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//    {
//		//������¼��ǰ�����ݳ��
//		cntss++;
//		if(cntss%50==0){
//		CAN_cmd_cap(11000);
//		}
//		if(cntss>=5001)
//			cntss=0;
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }

    while (1)
    {
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE)||robot_state.power_management_chassis_output==0)
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                //���Ϳ��Ƶ���
//							PID_calc(chassis_move.motor_speed_pid, chassis_move.motor_chassis[0].speed, set);
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //���̽Ƕ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	const static fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD }; //���ʻ�PID����

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
		//���̲���Ҫ��ȡ����������
//    //��ȡ��������̬��ָ��
//    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	//���ʻ�PID
	PID_init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	chassis_move.power_control.SPEED_MIN = 0.1f;
    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
		//����UI
		anglesr=abs((int)(chassis_move.chassis_yaw_motor->relative_angle*100));
		if(anglesr>157&&anglesr<314){
		anglesr=314-anglesr;
		}
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //���������̽Ƕ�ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

//    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
//    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
//    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
//    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		
		if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z)
		{
				*vx_set = 0.0f;
				*vy_set = 0.0f;
		}
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f,angle_set;
	fp64 relative_angle = 0.0f;
    //get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

		
    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�
			//������ڿ����ģʽ
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
//        //������תPID���ٶ�
//			if(fabs(chassis_move_control->chassis_relative_angle_set-chassis_move_control->chassis_yaw_motor->relative_angle)>=0.02f)��������
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
//			else 
//				chassis_move_control->wz_set=0;
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //���õ��̿��ƵĽǶ�
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //������ת�Ľ��ٶ�
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_BPIN)
    {
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		relative_angle = (fp64)chassis_move_control->chassis_yaw_motor->relative_angle-0.55;

//		if (relative_angle > PI)
//			relative_angle = -2 * PI + relative_angle;
//		else if (relative_angle < -PI)
//			relative_angle = 2 * PI + relative_angle;
//		else
//			relative_angle = relative_angle;

//		// �ǶȲ���
//		if (chassis_move_control->chassis_power_MAX == 120)
//			relative_angle += Power_120_AngleCompensation;
//		else if(chassis_move_control->chassis_power_MAX == 100)
//			relative_angle += Power_100_AngleCompensation;
//		else if(chassis_move_control->chassis_power_MAX == 80)
//			relative_angle += Power_80_AngleCompensation;
//		else if(chassis_move_control->chassis_power_MAX == 70)
//			relative_angle += Power_70_AngleCompensation;
//		else if(chassis_move_control->chassis_power_MAX == 60)
//			relative_angle += Power_60_AngleCompensation;
//		else if(chassis_move_control->chassis_power_MAX == 50)
//			relative_angle += Power_50_AngleCompensation;
//		else	
//			relative_angle += -0.2;


//		sin_yaw = arm_sin_f32((relative_angle));
//		cos_yaw = arm_cos_f32((relative_angle));

//		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
//		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;
//		chassis_move_control->chassis_relative_angle_set = rad_format(0.0);
//		
//		
		
		
//        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;			
		sin_yaw =(arm_sin_f32(-relative_angle));
        cos_yaw =(arm_cos_f32(-relative_angle));
			
		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
				
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
		//�����ٶ��趨
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	
    }
}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	wheel_speed[0] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.00f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.00f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.00f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.00f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


uint8_t cap_send_tmp=0;

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->chassis_power_MAX = robot_state.chassis_power_limit;
	chassis_move_control_loop->chassis_power_buffer = power_heat_data_t.chassis_power_buffer;
	 
	if(power_heat_data_t.chassis_power_buffer < 5)
		chassis_move_control_loop->chassis_power_MAX = 60;
		
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
//		static int32_t input_power;
//		static int8_t release_energy = 0;
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

	static int16_t last_key_Ctrl = 0;
	if(!last_key_Ctrl&&chassis_move_control_loop->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
	{
		feipo=!feipo;
	}	
	last_key_Ctrl = chassis_move_control_loop->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL;

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw����ֱ�ӷ���
        return;
    }
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
	
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
																chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		if (fabs(chassis_move_control_loop->power_control.speed[i]) < chassis_move_control_loop->power_control.SPEED_MIN)
		{
			chassis_move_control_loop->power_control.speed[i] = chassis_move_control_loop->power_control.SPEED_MIN;
		}
	}
	
	static int16_t last_key_F = 0;
	if(!last_key_F&&chassis_move_control_loop->chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
	{
		FCHO=!FCHO;
	}	
	last_key_F = chassis_move_control_loop->chassis_RC->key.v & KEY_PRESSED_OFFSET_F;
			
	if((get_capA.cap_voltage/1000) < 18.0)
		FCHO=0;
	
	
	uint16_t max_power_limit = 40;
	fp32 input_power = 0;		 // ���빦��(����������)
	fp32 scaled_motor_power[4];
	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  �˲������������ת��ΪŤ��
	fp32 k2 = 1.23e-07;						 // �Ŵ�ϵ��
	fp32 k1 = 1.453e-07;					 // �Ŵ�ϵ��
	fp32 constant = 4.081f;  //3508����Ļ�е���
	chassis_move_control_loop->power_control.POWER_MAX = 0; //���յ��̵������
	chassis_move_control_loop->power_control.forecast_total_power = 0; // Ԥ���ܹ���
	
	PID_calc(&chassis_move_control_loop->buffer_pid, chassis_move_control_loop->chassis_power_buffer,35); //ʹ��������ά����һ���ȶ��ķ�Χ,�����PIDû��Ҫ��ֲ�ҵģ�������һ������

	max_power_limit = chassis_move_control_loop->chassis_power_MAX;  //��ò���ϵͳ�Ĺ���������ֵ
	
	input_power = max_power_limit - chassis_move_control_loop->buffer_pid.out; //ͨ������ϵͳ�������
	
	chassis_move_control_loop->power_control.power_charge = input_power; //�������ݵ�����繦��
	
	if(chassis_move_control_loop->power_control.power_charge>100)		chassis_move_control_loop->power_control.power_charge =100; //�ο�������ư����������繦�ʣ�Ϫ�ذ��ӵ����ϲ�һ��
	
	if(chassis_move_control_loop->power_control.power_charge<30)		chassis_move_control_loop->power_control.power_charge =30;

	
	if(cap_send_tmp % 50 == 0)
		CAN_cmd_cap(chassis_move_control_loop->power_control.power_charge); // ���ó���ĳ�繦��
	cap_send_tmp++;

	if (get_capA.cap_voltage/1000 > 16) //�������ѹ����ĳ��ֵ(��ֹC620����)
	{
		if (FCHO)   //�������磬һ�������𲽼���or���or����or���£�chassis_move.key_CΪ�˴����г��翪������
		{
			if(feipo)
			chassis_move_control_loop->power_control.POWER_MAX = 275;
			else if(robot_state.robot_level < 5)
			chassis_move_control_loop->power_control.POWER_MAX = 90;
			else if(robot_state.robot_level > 4 && robot_state.robot_level < 8)
			chassis_move_control_loop->power_control.POWER_MAX = 110;
			else
			chassis_move_control_loop->power_control.POWER_MAX = 140;		
		}
		else
		{
			if(robot_state.robot_level < 5)
			{
			chassis_move_control_loop->power_control.POWER_MAX = input_power + 15;	
				if((get_capA.cap_voltage/1000) < 17.0)
				chassis_move_control_loop->power_control.POWER_MAX = input_power;
			}
			else
			{
			chassis_move_control_loop->power_control.POWER_MAX = input_power + 5;
			if((get_capA.cap_voltage/1000) < 20.0)
			chassis_move_control_loop->power_control.POWER_MAX = input_power + 2;
			if((get_capA.cap_voltage/1000) < 19.0)
			chassis_move_control_loop->power_control.POWER_MAX = input_power + 1;
			if((get_capA.cap_voltage/1000) < 18.0)
			chassis_move_control_loop->power_control.POWER_MAX = input_power;
			}
		}
	}
	else
	{
		chassis_move_control_loop->power_control.POWER_MAX = input_power;
	}

	for (uint8_t i = 0; i < 4; i++) // �������3508����Ĺ��ʺ��ܹ���
	{
		chassis_move_control_loop->power_control.forecast_motor_power[i] =
    		chassis_move_control_loop->motor_chassis[i].give_current * toque_coefficient * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm +
				k1 * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm +
				k2* chassis_move_control_loop->motor_chassis[i].give_current *chassis_move_control_loop->motor_chassis[i].give_current + constant;

		if (chassis_move_control_loop->power_control.forecast_motor_power < 0)  	continue; // ���Ը���
		
		chassis_move_control_loop->power_control.forecast_total_power += chassis_move_control_loop->power_control.forecast_motor_power[i];
	}
	
	if(feipo)//����ģʽ��ǰ���ַ��书�����ں����֣������ɵ�
    {
	    chassis_move_control_loop->power_control.forecast_motor_power[0] = (chassis_move_control_loop->power_control.forecast_total_power/10)*1.5f;
	    chassis_move_control_loop->power_control.forecast_motor_power[1] = (chassis_move_control_loop->power_control.forecast_total_power/10)*1.5f;
	    chassis_move_control_loop->power_control.forecast_motor_power[2] = (chassis_move_control_loop->power_control.forecast_total_power/10)*3.5f;
	    chassis_move_control_loop->power_control.forecast_motor_power[3] = (chassis_move_control_loop->power_control.forecast_total_power/10)*3.5f;
    }
	
	if (chassis_move_control_loop->power_control.forecast_total_power > chassis_move_control_loop->power_control.POWER_MAX) // ������ģ��˥��
	{
		fp32 power_scale = chassis_move_control_loop->power_control.POWER_MAX / chassis_move_control_loop->power_control.forecast_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_motor_power[i] = chassis_move_control_loop->power_control.forecast_motor_power[i] * power_scale; // ���˥����Ĺ���
			
			if (scaled_motor_power[i] < 0)		continue;

			fp32 b = toque_coefficient * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k1 * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

			if (chassis_move_control_loop->motor_chassis[i].give_current> 0)  //���ⳬ��������
			{					
				chassis_move_control_loop->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);  
				if (chassis_move_control_loop->power_control.MAX_current[i] > 16000)
				{
					chassis_move_control_loop->motor_chassis[i].give_current = 16000;
				}
				else
					chassis_move_control_loop->motor_chassis[i].give_current = chassis_move_control_loop->power_control.MAX_current[i];
			}
			else
			{
				chassis_move_control_loop->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
				if (chassis_move_control_loop->power_control.MAX_current[i] < -16000)
				{
					chassis_move_control_loop->motor_chassis[i].give_current = -16000;
				}
				else
					chassis_move_control_loop->motor_chassis[i].give_current = chassis_move_control_loop->power_control.MAX_current[i];
			}
		}
	}
	
//		chassis_move_control_loop->power_control.K = 480;
//		chassis_move_control_loop->power_control.POWER_MAX = 80;
////		input_power = 4500;
//	
//	    //����pid
//    for (i = 0; i < 4; i++)
//    {
//        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
//				chassis_move_control_loop->power_control.speed[i]=chassis_move_control_loop->motor_chassis[i].speed;
//				if(fabs(chassis_move_control_loop->power_control.speed[i]) < 0.01)
//				{
//					chassis_move_control_loop->power_control.speed[i] = 0.01;				
//				}
//				chassis_move_control_loop->power_control.totalSpeed+=fabs(chassis_move_control_loop->power_control.speed[i]);
//    }
//		//�����ܵ���
//		for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->motor_chassis[i].give_current=(fp32)(chassis_move_control_loop->motor_speed_pid[i].out);
//				chassis_move_control_loop->power_control.current[i]=chassis_move_control_loop->motor_chassis[i].give_current;
//				chassis_move_control_loop->power_control.totalCurrentTemp+=fabs(chassis_move_control_loop->power_control.current[i]);
//    }
//		//���㵥�����������
//		for(i=0;i<4;i++)
//		{
//				chassis_move_control_loop->power_control.MAX_current[i]=(chassis_move_control_loop->power_control.K*chassis_move_control_loop->power_control.current[i]/chassis_move_control_loop->power_control.totalCurrentTemp)
//				*(chassis_move_control_loop->power_control.POWER_MAX)/(4*fabs(chassis_move_control_loop->power_control.speed[i]));
//				//�������᲻ͬ��Ҫ�ĵ�����ͬ�� 
//		}
//		chassis_move_control_loop->power_control.totalCurrentTemp=0;
//		chassis_move_control_loop->power_control.totalSpeed=0;
//		//����ֵ
//		for(i=0;i<4;i++)
//		{   
//				if(abs(chassis_move_control_loop->motor_chassis[i].give_current)>=abs(chassis_move_control_loop->power_control.MAX_current[i]))
//				{			
//						chassis_move_control_loop->motor_chassis[i].give_current=chassis_move_control_loop->power_control.MAX_current[i];
//				}
//				else
//				{   
//						chassis_move_control_loop->motor_chassis[i].give_current=(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);		
//				}
//		}

}

