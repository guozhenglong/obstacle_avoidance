#include "multi_drone_safe_ctrl/safe_ctrl.h"
namespace Safe_Ctrl
{
	safe_ctrl::safe_ctrl(ros::NodeHandle& nh, ros::NodeHandle& pnh) :nh_(nh)
	{
		pnh.param("debug", debug, true);
		pnh.param("Vmax", Vmax, 10.0);
        pnh.param("l", l, 0.2);
        pnh.param("rm", rm, 0.4);
        pnh.param("ra", ra, 1.0);
        pnh.param("rh", rh, 10.0);
        pnh.param("e", e, 0.01);
        pnh.param("rs", rs, 0.01);
        pnh.param("kx", kx, 1.0);
        pnh.param("ky", ky, 1.0);
        pnh.param("k_m", k_m,1.0);
        pnh.param("x_b", x_b, 20.0);
        pnh.param("y_b", y_b, 20.0);
        pnh.param("Hz", Hz, 20.0);
        pnh.param("gain_cmd",gain_cmd,0.5);
        ros::Rate loopRate(Hz);
        x_h = x_b / 2.0;
        y_h = y_b / 2.0;

        geo_half_x = x_h;
        geo_half_y = y_h;

        geo_x_min = 0.0;
        geo_y_min = 0.0;

        // double dist_test = -10.0;
        // double sig_test = 0.0;
        // geometry_msgs::Point cmd_geo_test;
        // for(int i=0;i<40;i++)
        // {
        //     safe_ctrl::Vb_cmd(cmd_geo_test, dist_test, kx,1);
        //     //sig_test = safe_ctrl::grad_s_m(dist_test,rs);
        //     dist_test = dist_test + 0.5;
        //     cout<<"Seq=:"<<i<<endl;
        //     cout<<"R=:"<<endl<<cmd_geo_test<<endl;
        // }

        pub_vel_cmd1 = nh_.advertise<geometry_msgs::Twist>("/uav_cmd_vel1", 1);
        pub_vel_cmd2 = nh_.advertise<geometry_msgs::Twist>("/uav_cmd_vel2", 1);
        get_cmd_input1 = nh_.subscribe("/cmd_input1", 1, &safe_ctrl::CmdInputCallBack1, this);
        get_cmd_input2 = nh_.subscribe("/cmd_input2", 1, &safe_ctrl::CmdInputCallBack2, this);
        get_navdata1 = nh_.subscribe("/navdata_uav1", 2, &safe_ctrl::NavdataCallBack1, this);
        get_navdata2 = nh_.subscribe("/navdata_uav2", 2, &safe_ctrl::NavdataCallBack2, this);

        debug_pub =nh_.advertise<multi_drone_safe_ctrl::DebugData>("debug_topic", 1);

        while(ros::ok())
        {

            // if (NavisNewMsg1 && NavisNewMsg2)
            // {
                safe_ctrl::Vm_cmd(cmd_aviodance_1, pos_nav_vec1, pos_nav_vec2);
                safe_ctrl::Vm_cmd(cmd_aviodance_2, pos_nav_vec2, pos_nav_vec1);
            // }
            // else
            // {
            //     cmd_aviodance_1.x = 0.0;
            //     cmd_aviodance_1.y = 0.0;
            //     cmd_aviodance_1.z = 0.0;
            //     cmd_aviodance_2.x = 0.0;
            //     cmd_aviodance_2.y = 0.0;
            //     cmd_aviodance_2.z = 0.0;
            // }


            // if (NavisNewMsg1)
            // {
                geo_cmd1.x = cmd_geo_x1.x + cmd_geo_y1.x;
                geo_cmd1.y = cmd_geo_x1.y + cmd_geo_y1.y;
                geo_cmd1.z = cmd_geo_x1.z + cmd_geo_y1.z;

                
                // output_cmd_idea1.x = input_cmd1.linear.x + geo_cmd1.x - cmd_aviodance_1.x;
                // output_cmd_idea1.y = input_cmd1.linear.y + geo_cmd1.y - cmd_aviodance_1.y;
                // output_cmd_idea1.z = input_cmd1.linear.z + geo_cmd1.z - cmd_aviodance_1.z;
                
                // for single static obstacle test.
                //***********************************//
                output_cmd_idea1.x = input_cmd1.linear.x - cmd_aviodance_1.x;
                output_cmd_idea1.y = input_cmd1.linear.y - cmd_aviodance_1.y;
                output_cmd_idea1.z = input_cmd1.linear.z - cmd_aviodance_1.z;
                //***********************************//


                // }
                // else
                // {
                //     if (debug)
                //     {
                //         cout << "No Update!" << endl;
                //     }
                //     output_cmd_idea1.x = input_cmd1.linear.x;
                //     output_cmd_idea1.y = input_cmd1.linear.y;
                //     output_cmd_idea1.z = input_cmd1.linear.z;
                // }

                // if (NavisNewMsg2)
                // {
                geo_cmd2.x = cmd_geo_x2.x + cmd_geo_y2.x;
                geo_cmd2.y = cmd_geo_x2.y + cmd_geo_y2.y;
                geo_cmd2.z = cmd_geo_x2.z + cmd_geo_y2.z;
                output_cmd_idea2.x = input_cmd2.linear.x + geo_cmd2.x - cmd_aviodance_2.x;
                output_cmd_idea2.y = input_cmd2.linear.y + geo_cmd2.y - cmd_aviodance_2.y; 
                output_cmd_idea2.z = input_cmd2.linear.z + geo_cmd2.z - cmd_aviodance_2.z;
            // }
            // else
            // {
            //     if (debug)
            //     {
            //         cout << "No Update!" << endl;
            //     }
            //     output_cmd_idea2.x = input_cmd2.linear.x;
            //     output_cmd_idea2.y = input_cmd2.linear.y;
            //     output_cmd_idea2.z = input_cmd2.linear.z;
            // }



            debug_msg.header.frame_id="debug";
            debug_msg.header.stamp=ros::Time::now();
            debug_msg.nav1 = navdata_msg1;
            debug_msg.nav2 = navdata_msg2;
            debug_msg.in_cmd1 = input_cmd1;
            debug_msg.in_cmd2 = input_cmd2;
            debug_msg.out_cmd_idea1 = output_cmd_idea1;
            debug_msg.out_cmd_idea2 = output_cmd_idea2;
            debug_msg.cmd_geo1 = geo_cmd1;
            debug_msg.cmd_geo2 = geo_cmd2;
            debug_msg.cmd_aviod1 = cmd_aviodance_1;
            debug_msg.cmd_aviod2 = cmd_aviodance_2;

            safe_ctrl::sat_vel_vec(output_cmd_idea1, output_cmd_idea1, Vmax);
            safe_ctrl::sat_vel_vec(output_cmd_idea2, output_cmd_idea2, Vmax);

            output_cmd1.linear.x = output_cmd_idea1.x;
            output_cmd1.linear.y = output_cmd_idea1.y;
            output_cmd1.linear.z = output_cmd_idea1.z;
            output_cmd1.angular = input_cmd1.angular;

            output_cmd2.linear.x = output_cmd_idea2.x;
            output_cmd2.linear.y = output_cmd_idea2.y;
            output_cmd2.linear.z = output_cmd_idea2.z;
            output_cmd2.angular = input_cmd2.angular;
            
            debug_msg.out_cmd1 = output_cmd1;
            debug_msg.out_cmd2 = output_cmd2;
            debug_pub.publish(debug_msg);
            if (debug)
            {
                cout << "output_cmd 1:" << endl;
                cout << output_cmd1 << endl;
                cout << "output_cmd 2:" << endl;
                cout << output_cmd2 << endl;
            }

            pub_vel_cmd1.publish(output_cmd1);
            pub_vel_cmd2.publish(output_cmd2);
            ros::spinOnce();
            loopRate.sleep();
        }
	}

    void safe_ctrl::CmdInputCallBack1(const geometry_msgs::Twist& msg)
    { 
            input_cmd1 = msg;
            if (debug)
                cout << "input_cmd:" << endl << msg << endl;
    }

	// void safe_ctrl::NavdataCallBack1(const aruco_navigation::Navdata_aruco& msg)
	// {
    //     if (msg.IsNewNav)
    //     {
    //         navdata_msg1 = msg;
    //         NavisNewMsg1 = true;
    //         pos_current1 = navdata_msg1.pose.position;
    //         vel_current1 = navdata_msg1.velocity;
    //         euler1 = navdata_msg1.euler_angle;
    //         safe_ctrl::nav_vec(pos_nav_vec1, pos_current1, vel_current1);
    //         if (debug)
    //         {
    //             cout << "NavisNewMsg:New Update!" << endl;
    //             cout << "pos_current:" << endl;
    //             cout << pos_current1 << endl;
    //         }

    //         safe_ctrl::dis_p2l_vec(dis_geo_x1, pos_nav_vec1, geo_half_x, 1);
    //         safe_ctrl::dis_p2l_vec(dis_geo_y1, pos_nav_vec1, geo_half_y, 0);
    //         safe_ctrl::Vb_cmd(cmd_geo_x1, dis_geo_x1, kx, 1);
    //         safe_ctrl::Vb_cmd(cmd_geo_y1, dis_geo_y1, ky, 0);
    //     }
    //     else
    //     {
    //         NavisNewMsg1 = false;
    //     } 
	// }

    void safe_ctrl::NavdataCallBack1(const nav_msgs::Odometry &msg)
    {
        // if (msg.IsNewNav)
        // {
            navdata_msg1 = msg;
            // NavisNewMsg1 = true;
            pos_current1 = navdata_msg1.pose.pose.position;
            vel_current1.x = navdata_msg1.twist.twist.linear.x;
            vel_current1.y = navdata_msg1.twist.twist.linear.y;
            vel_current1.z = navdata_msg1.twist.twist.linear.z;
            // euler1 = navdata_msg1.euler_angle;
            safe_ctrl::nav_vec(pos_nav_vec1, pos_current1, vel_current1);
            if (debug)
            {
                cout << "NavisNewMsg:New Update!" << endl;
                cout << "pos_current:" << endl;
                cout << pos_current1 << endl;
            }

            safe_ctrl::dis_p2l_vec(dis_geo_x1, pos_nav_vec1, geo_half_x, 1);
            safe_ctrl::dis_p2l_vec(dis_geo_y1, pos_nav_vec1, geo_half_y, 0);
            safe_ctrl::Vb_cmd(cmd_geo_x1, dis_geo_x1, kx, 1);
            safe_ctrl::Vb_cmd(cmd_geo_y1, dis_geo_y1, ky, 0);
        // }
        // else
        // {
        //     NavisNewMsg1 = false;
        // }
    }
    void safe_ctrl::CmdInputCallBack2(const geometry_msgs::Twist &msg)
    {
        input_cmd2 = msg;
        if (debug)
            cout << "input_cmd:" << endl
                 << msg << endl;
    }

    // void safe_ctrl::NavdataCallBack2(const aruco_navigation::Navdata_aruco &msg)
    // {
    //     if (msg.IsNewNav)
    //     {
    //         navdata_msg2 = msg;
    //         NavisNewMsg2 = true;
    //         pos_current2 = navdata_msg2.pose.position;
    //         vel_current2 = navdata_msg2.velocity;
    //         euler2 = navdata_msg2.euler_angle;
    //         safe_ctrl::nav_vec(pos_nav_vec2, pos_current2, vel_current2);
    //         if (debug)
    //         {
    //             cout << "NavisNewMsg:New Update!" << endl;
    //             cout << "pos_current:" << endl;
    //             cout << pos_current2 << endl;
    //         }
    //         safe_ctrl::dis_p2l_vec(dis_geo_x2, pos_nav_vec2, geo_half_x, 1);
    //         safe_ctrl::dis_p2l_vec(dis_geo_y2, pos_nav_vec2, geo_half_y, 0);
    //         safe_ctrl::Vb_cmd(cmd_geo_x2, dis_geo_x2, kx, 1);
    //         safe_ctrl::Vb_cmd(cmd_geo_y2, dis_geo_y2, ky, 0);
    //     }
    //     else
    //     {
    //         NavisNewMsg2 = false;
    //     }
    // }
    void safe_ctrl::NavdataCallBack2(const nav_msgs::Odometry &msg)
    {
        // if (msg.IsNewNav)
        // {
            navdata_msg2 = msg;
            // NavisNewMsg2 = true;
            pos_current2 = navdata_msg2.pose.pose.position;
            vel_current2.x = navdata_msg2.twist.twist.linear.x;
            vel_current2.y = navdata_msg2.twist.twist.linear.y;
            vel_current2.z = navdata_msg2.twist.twist.linear.z;

            // for single static obstacle test.
            //***********************************//
            pos_current2.x = 0.0;
            pos_current2.y = 0.0;
            pos_current2.z = 0.0;
            vel_current2.x = 0.0;
            vel_current2.y = 0.0;
            vel_current2.z = 0.0;
            //***********************************//

            // euler2 = navdata_msg2.euler_angle;
            safe_ctrl::nav_vec(pos_nav_vec2, pos_current2, vel_current2);
            if (debug)
            {
                cout << "NavisNewMsg:New Update!" << endl;
                cout << "pos_current:" << endl;
                cout << pos_current2 << endl;
            }
            safe_ctrl::dis_p2l_vec(dis_geo_x2, pos_nav_vec2, geo_half_x, 1);
            safe_ctrl::dis_p2l_vec(dis_geo_y2, pos_nav_vec2, geo_half_y, 0);
            safe_ctrl::Vb_cmd(cmd_geo_x2, dis_geo_x2, kx, 1);
            safe_ctrl::Vb_cmd(cmd_geo_y2, dis_geo_y2, ky, 0);
        // }
        // else
        // {
        //     NavisNewMsg2 = false;
        // }
    }
    void safe_ctrl::nav_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos,geometry_msgs::Point& vel)
    {
        output.x = pos.x + l*vel.x;
        output.y = pos.y + l*vel.y;
        output.z = pos.z + l*vel.z;

    }

    void safe_ctrl::dis_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos1,geometry_msgs::Point& pos2)
    {
        output.x = pos1.x - pos2.x;
        output.y = pos1.y - pos2.y;
        output.z = pos1.z - pos2.z;
    }

    void safe_ctrl::dis_p2l_vec(double& output, geometry_msgs::Point& pos1,double& line,int type)
    {
        if (type == 1)
        {
            output = pos1.x - line;
        }
        if (type == 0)
        {
            output = pos1.y - line;
        }

    }

    void safe_ctrl::Vm_cmd(geometry_msgs::Point &output_vel, geometry_msgs::Point &nav_vec1, geometry_msgs::Point &nav_vec2)
    {
        geometry_msgs::Point dis_vec;
        double dis;
        double rm_m2, rm_d2;

        safe_ctrl::dis_vec(dis_vec, nav_vec1, nav_vec2);
        dis = safe_ctrl::norm_vec(dis_vec);
        rm_m2 = 2.0*rm;
        rm_d2 = dis/rm_m2;
        double f = k_m*(safe_ctrl::sig(dis,rm_m2,ra));
        double f_grad = k_m*(safe_ctrl::grad_sig(dis,rm_m2,ra));
        double g = (1 + e) * dis - 2 * rm * (safe_ctrl::s_m(rm_d2, rs));
        double g_grad = (1 + e) - (safe_ctrl::grad_s_m(rm_d2, rs));
        double V = (f_grad*g-f*g_grad)/(g*g);
        // cout<<"f=:"<<f<<endl;
        // cout<<"f_grad=:"<<f_grad<<endl;
        // cout<<"g=:"<<g<<endl;
        // cout<<"g_grad=:"<<g_grad<<endl;
        // cout<<"V=:"<<V<<endl;

        output_vel.x = dis_vec.x / dis *V;
        output_vel.y = dis_vec.y / dis * V;
        //output_vel.z = dis_vec.z / dis * V;
        output_vel.z = 0.0;
    }

    void safe_ctrl::Vb_cmd(geometry_msgs::Point& output_vel, double& dis, double& k,int type)
    {
        int sgn;
        if(dis >= 0.0)
            sgn = -1;
        else
            sgn = 1;
        dis = abs(dis);
        double dis_h = rh - dis;
        double f = k*(safe_ctrl::sig(dis_h,rm,ra));
        double f_grad = - k*(safe_ctrl::grad_sig(dis_h,rm,ra));
        double tmp = (rh-rm)/(dis+e);
        double g = (rh-rm) - dis*(safe_ctrl::s_m(tmp,rs));
        double g_grad = -(safe_ctrl::s_m(tmp,rs)) + dis*(safe_ctrl::grad_s_m(tmp,rs))*(rh-rm)/((dis+e)*(dis+e)) ; 
        double V_= f/g;
        double V = (f_grad*g-f*g_grad)/(g*g);
        // cout<<"dis=:"<<dis<<endl;
        // cout<<"dis_h=:"<<dis_h<<endl; 
        // cout<<"f=:"<<f<<endl;
        // cout<<"f_grad=:"<<f_grad<<endl;
        // cout<<"g=:"<<g<<endl;
        // cout<<"g_grad=:"<<g_grad<<endl;
        // cout<<"V_=:"<<V_<<endl;
        // cout<<"V=:"<<V<<endl;
        if(type == 1)
        {
            output_vel.x = sgn*V;
            output_vel.y = 0.0;
            output_vel.z = 0.0;
        }
        if(type == 0)
        {
            output_vel.x = 0.0;
            output_vel.y = sgn*V;
            output_vel.z = 0.0;
        }


    }
    double safe_ctrl::sig(double& x, double& d1,double& d2)
    {
        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));
        // cout<<"x=:"<<x<<"   d1=:"<<d1<<"    d2=:"<<d2<<endl;
        // cout<<"A=:"<<A<<"   B=:"<<B<<"    C=:"<<C<<"    D=:"<<D<<endl;
        if (x<d1)
            return 1.0;
        else if (x>d2)
            return 0.0;
        else
            return A*x*x*x+B*x*x+C*x+D;
    } 

    double safe_ctrl::grad_sig(double& x, double& d1,double& d2)
    {
        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));
        // cout<<"x=:"<<x<<"   d1=:"<<d1<<"    d2=:"<<d2<<endl;
        // cout<<"A=:"<<A<<"   B=:"<<B<<"    C=:"<<C<<"    D=:"<<D<<endl;
        if (x<d1)
            return 0.0;
        else if (x>d2)
            return 0.0;
        else
            return 3*A*x*x+2*B*x+C;
    }

    double safe_ctrl::s_m(double& x, double& rs)
    {
        double x2 = 1+rs*(1/tan(67.5/180*PI));
        double x1 = x2 - rs*sin(45/180*PI);
        // cout<<"x=:"<<x<<"   rs=:"<<rs<<endl;
        // cout<<"x1=:"<<x1<<"   x2=:"<<x2<<endl;
        if (x>=0 && x<x1)
            return x;
        else if (x >x2)
            return 1.0;
        else
            return (1-rs)+sqrt(rs*rs-(x-x2)*(x-x2));
    }

    double safe_ctrl::grad_s_m(double& x, double& rs)
    {
        double x2 = 1+rs*(1/tan(67.5/180*PI));
        double x1 = x2 - rs*sin(45/180*PI);
        // cout<<"x=:"<<x<<"   rs=:"<<rs<<endl;
        // cout<<"x1=:"<<x1<<"   x2=:"<<x2<<endl;
        if (x>=0 && x<x1)
            return 1.0;
        else if (x >x2)
            return 0.0;
        else
            return (x2-x)/sqrt(rs*rs-(x-x2)*(x-x2));
    }


    double safe_ctrl::norm_vec(geometry_msgs::Point& input)
    {
        double x = input.x;
        double y = input.y;
        return sqrt(x*x+y*y);
    }

    void safe_ctrl::sat_vel_vec(geometry_msgs::Point& output, geometry_msgs::Point& input, double& Vmax)
    {
        double norm_vec =  safe_ctrl::norm_vec(input);
        if (norm_vec < Vmax)
            output = input;
        else
        {
            output.x = input.x / norm_vec * Vmax;
            output.y = input.y / norm_vec * Vmax;
            //output.z = input.z / norm_vec * Vmax;
            output.z = input.z;
        }

    }

}
