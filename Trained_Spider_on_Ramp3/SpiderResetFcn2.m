% Zaid H. Jaber <z.h.jaber@outlook.com>
% Princess Sumaya University for Technology
% Created: Sep 2023
% Available online: https://github.com/ZaidHJaber/Four-legged-Spider-Robot-RL-locomotion.git

% Helper function to reset spider robot simulation with different initial conditions


function in = SpiderResetFcn2(in)
    
    min_body_height = 0.010; %10mm
    max_body_height = 0.040; %80mm
    max_speed_x = 0.05;
    max_speed_z = 0.025;
 
    % % Randomization Parameters
    if rand > 0.3
        d2r = pi/180;
        z_body1=min_body_height+ (max_body_height -min_body_height)*rand;
        X_displacment = rand* (40 - -15) + -15;
        angles = InverseKinematics2(X_displacment,z_body1);
        tibiaRL_ang = -angles(1);
        femurRL_ang = -angles(2);
        coxaRL_ang  = -angles(3);

        X_displacment = rand* (40 - -15) + -15;
        z_body2=min_body_height+ (max_body_height -min_body_height)*rand;
        angles = InverseKinematics2(X_displacment,z_body2);
        tibiaRR_ang = angles(1);
        femurRR_ang = angles(2);
        coxaRR_ang  = angles(3);

        X_displacment = rand* (40 - -15) + -15;
        z_body3=min_body_height+ (max_body_height -min_body_height)*rand;
        angles = InverseKinematics2(X_displacment,z_body3);
        tibiaFL_ang = angles(1);
        femurFL_ang = angles(2);
        coxaFL_ang  = angles(3);
        
        X_displacment = rand* (40 - -15) + -15;
        z_body4=min_body_height+ (max_body_height -min_body_height)*rand;
        angles = InverseKinematics2(X_displacment,z_body4);
        tibiaFR_ang = -angles(1);
        femurFR_ang = -angles(2);
        coxaFR_ang  = -angles(3);

        z_body = max([z_body1,z_body2,z_body3,z_body4]);
            
        % Randomize body velocities
        vx = 2 * max_speed_x * (rand-0.5);
        vy = 2 * max_speed_z * (rand-0.5);
    else
        d2r = pi/180;
        z_body=0.02;
        angles = InverseKinematics(z_body);
        tibiaRL_ang = -angles(1);
        tibiaRR_ang = angles(1);
        tibiaFL_ang = angles(1);
        tibiaFR_ang = -angles(1);
        femurRL_ang = -angles(2);
        femurRR_ang = angles(2);
        femurFL_ang = angles(2);
        femurFR_ang = -angles(2);
        coxaRL_ang  = -angles(3);
        coxaRR_ang  = angles(3);
        coxaFL_ang  = angles(3);
        coxaFR_ang  = -angles(3);
        vx = 0;
        vz = 0;
    end
    in = setVariable(in,'init_6DOF_highet',z_body);
    in = setVariable(in,'init_tibiaRL_ang',tibiaRL_ang);
    in = setVariable(in,'init_tibiaRR_ang',tibiaRR_ang);
    in = setVariable(in,'init_tibiaFL_ang',tibiaFL_ang);
    in = setVariable(in,'init_tibiaFR_ang',tibiaFR_ang);

    in = setVariable(in,'init_femurRL_ang',femurRL_ang);
    in = setVariable(in,'init_femurRR_ang',femurRR_ang);
    in = setVariable(in,'init_femurFL_ang',femurFL_ang);
    in = setVariable(in,'init_femurFR_ang',femurFR_ang);

    in = setVariable(in,'init_coxaRL_ang',coxaRL_ang);
    in = setVariable(in,'init_coxaRR_ang',coxaRR_ang);
    in = setVariable(in,'init_coxaFL_ang',coxaFL_ang);
    in = setVariable(in,'init_coxaFR_ang',coxaFR_ang);


    in = setVariable(in,'vx_init',vx);
    in = setVariable(in,'vz_init',vz);

end