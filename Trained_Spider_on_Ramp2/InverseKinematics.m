function [angles] = InverseKinematics(dh)
        %Desired_height (dh) of the SV3 Axel 
        dh=dh*1000;
        Xt=60;Yt=60; %60
        Z_offset = (dh+25);
        D = sqrt((Xt)^2 + (Yt)^2);
        d = D - 27.5;
        R = sqrt((d)^2 + (Z_offset)^2);
        Theta3 = -(atan( Xt/Yt )); %coxa angle
        Theta2 =pi/2-(acos((55^2+R^2-80^2)/(2*55*R))+acos(Z_offset/R)) ; %Fumer
        Theta1 =pi/2-(acos((55^2+80^2-R^2)/(2*55*80)));  %Tibia
        angles = [Theta1,Theta2,Theta3];
        
end


