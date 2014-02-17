% clear q_text % Remove this variable if it exists
% clc
% 
% disp('TEO Humanoid Robot - Plot TEO')
% disp('Robotics Lab, Universidad Carlos III de Madrid.')
% disp('<a href = "http://roboticslab.uc3m.es">http://roboticslab.uc3m.es</a>')
% disp(' ')
% 
% 
% if ~exist('q')
%     disp('There is not joint values to plot (No "q": variable). TEO will be plot only in default position')
%     selection=input('Do you want to continue? y/n [y]: ','s');
%     if (selection=='n' | selection=='N')
%         return
%     elseif  (selection=='y' | selection=='Y')
%         disp('Please wait some seconds...')
%         disp(' ')
%         break
%     else
%         disp(['Error: ', selection, ' is not an option. TEO will be plot anyway']);
%         disp(' ')
%         disp('Please wait some seconds...')
%         disp(' ')
%     end
% else
%     disp(' ')
%     disp('Please wait some seconds...')
%     disp(' ')
% end;
% 


function TEO_Plot_function (trajectory, q, TEO, h)
% % LIBRERIA
% 
% TEO = TEO_structure('numeric', 'rad', 'm');

% MATRIZ DE CONVERSION PARA PLOT
Rotation_Matrix_Plot_Legs =  r2t([0 -1 0;...
                        0 0 1;...
                        -1 0 0]);
                    
Rotation_Matrix_Plot_Arms =  r2t([1 0 0;...
                        0 -1 0;...
                        0 0 -1]);                  

%LEGS
%link_lengths=TEO.legs.link_lengths;
qplot = generate_symbolic_vector('theta', 6);
[right_leg_floating.joint, right_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(TEO.legs.link_lengths(2:5), qplot);
[left_leg_floating.joint, left_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(TEO.legs.link_lengths(2:5), qplot);

for i=1:size(qplot,1)
TEO_right_leg(i) = Link(double([right_leg_floating.joint(i).theta-qplot(i), right_leg_floating.joint(i).d, right_leg_floating.joint(i).a, right_leg_floating.joint(i).alpha, 0]));
TEO_left_leg(i) = Link(double([left_leg_floating.joint(i).theta-qplot(i), -left_leg_floating.joint(i).d, left_leg_floating.joint(i).a, left_leg_floating.joint(i).alpha, 0]));
end;

%ARMS
% generate_humanoid_arms_kinematics  ([TEO.chest.link_lengths], [TEO.arms.link_lengths]);
[humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(TEO.arms.link_lengths(2:3), qplot);

for i=1:size(qplot,1)
TEO_right_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
TEO_left_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
end;

%TORSO
%generate_humanoid_torso_kinematics ([TEO.waist.link_lengths], [TEO.torso.link_lengths]);
qplot = generate_symbolic_vector('theta', 2);
[humanoid_torso.joint, humanoid_torso.n_joints] = humanoid_torso_DH_parameters(TEO.torso.link_lengths(1), qplot);

for i=1:size(qplot,1)
TEO_torso(i) = Link(double([humanoid_torso.joint(i).theta-qplot(i), humanoid_torso.joint(i).d, humanoid_torso.joint(i).a, humanoid_torso.joint(i).alpha, 0]));
end;

% TEO_right_leg(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
%     'I', [0, 0.35, 0, 0, 0, 0], ...
%     'r', [0, 0, 0], ...
%     'm', 0, ...
%     'Jm', 200e-6, ...
%     'G', -62.6111, ...
%     'B', 1.48e-3, ...
%     'Tc', [0.395 -0.435], ...
%     'qplotlim', [-160 160]*deg );

TEO_right_leg = SerialLink(TEO_right_leg, 'name', 'TEO_LR', 'base', transl(TEO.legs.right.joint(1).origin)*Rotation_Matrix_Plot_Legs, 'humanoid_structure', TEO);
TEO_left_leg = SerialLink(TEO_left_leg, 'name', 'TEO_LL', 'base', transl(TEO.legs.left.joint(1).origin)*Rotation_Matrix_Plot_Legs, 'humanoid_structure', TEO);

TEO_right_arm = SerialLink(TEO_right_arm, 'name', 'TEO_AR', 'base', transl(TEO.arms.right.joint(1).origin)*Rotation_Matrix_Plot_Arms, 'humanoid_structure', TEO);
TEO_left_arm = SerialLink(TEO_left_arm, 'name', 'TEO_AL', 'base', transl(TEO.arms.left.joint(1).origin)*Rotation_Matrix_Plot_Arms, 'humanoid_structure', TEO);

TEO_torso = SerialLink(TEO_torso, 'name', 'TEO_T', 'base', transl(TEO.origin), 'humanoid_structure', TEO); % NO ES EL ORIGEN, O SI?

figure(50), title('TEO Humanoid Robot - Universidad Carlos III de Madrid','color','blue','FontSize',16)

TEO_right_leg.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')

set(gca,'XDir','reverse');
set(gca,'YDir','reverse');
axis([-1.25 1.25 -1.25 1.25 -0.5 2])


hold on;
TEO_left_leg.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')%,'nowrist')

TEO_right_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')
TEO_left_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')

TEO_torso.plot(zeros(1,2),'nobase','noshadow','nojaxes','noname','nowrist')


% 
% %%%DOMINGOOOOOOOOOOOOOOOOOOOOOO
% %HIP
%     TEO.hip.origin(1)=TEO.legs.right.joint(1).origin(1);
%     TEO.hip.origin(2)=TEO.legs.right.joint(1).origin(2)+TEO.legs.link_lengths(1);
%     TEO.hip.origin(3)=TEO.legs.right.joint(1).origin(3);
%     
%     hiplink = line(TEO_right_leg.lineopt{:});
%     xlip(1)=TEO.legs.right.joint(1).origin(1);
%     ylip(1)=TEO.legs.right.joint(1).origin(2);
%     zlip(1)=TEO.legs.right.joint(1).origin(3);
%     xlip(2)=TEO.hip.origin(1);
%     ylip(2)=TEO.hip.origin(2);
%     zlip(2)=TEO.hip.origin(3);    
%     xlip(3)=TEO.legs.left.joint(1).origin(1);
%     ylip(3)=TEO.legs.left.joint(1).origin(2);
%     zlip(3)=TEO.legs.left.joint(1).origin(3);    
%     set(hiplink,'xdata', xlip, 'ydata', ylip, 'zdata', zlip);
%     
% %WAIST
%     waistlink=line(TEO_torso.lineopt{:});
%     xwaist(1)=TEO.hip.origin(1);
%     ywaist(1)=TEO.hip.origin(2);
%     zwaist(1)=TEO.hip.origin(3);
%     xwaist(2)=TEO.waist.joint(1).origin(1);
%     ywaist(2)=TEO.waist.joint(1).origin(2);
%     zwaist(2)=TEO.waist.joint(1).origin(3);
%     set(waistlink,'xdata', xwaist, 'ydata', ywaist, 'zdata', zwaist);
% 
% %CLAVICLE
%     TEO.clavicle.origin(1)=TEO.arms.right.joint(1).origin(1);
%     TEO.clavicle.origin(2)=TEO.arms.right.joint(1).origin(2)+TEO.arms.link_lengths(1);
%     TEO.clavicle.origin(3)=TEO.arms.right.joint(1).origin(3);
%     
%     claviclelink=line(TEO_right_arm.lineopt{:});
%     xclavicle(1)=TEO.arms.right.joint(1).origin(1);
%     yclavicle(1)=TEO.arms.right.joint(1).origin(2);
%     zclavicle(1)=TEO.arms.right.joint(1).origin(3);
%     xclavicle(2)=TEO.clavicle.origin(1);
%     yclavicle(2)=TEO.clavicle.origin(2);
%     zclavicle(2)=TEO.clavicle.origin(3);    
%     xclavicle(3)=TEO.arms.left.joint(1).origin(1);
%     yclavicle(3)=TEO.arms.left.joint(1).origin(2);
%     zclavicle(3)=TEO.arms.left.joint(1).origin(3);    
%     set(claviclelink,'xdata', xclavicle, 'ydata', yclavicle, 'zdata', zclavicle);
%     
% %TORSO
%     torsolink=line(TEO_torso.lineopt{:});
%     xtorso(1)=TEO.waist.joint(1).origin(1);
%     ytorso(1)=TEO.waist.joint(1).origin(2);
%     ztorso(1)=TEO.waist.joint(1).origin(3);
%     xtorso(2)=TEO.clavicle.origin(1);
%     ytorso(2)=TEO.clavicle.origin(2);
%     ztorso(2)=TEO.clavicle.origin(3);
%     set(torsolink,'xdata', xtorso, 'ydata', ytorso, 'zdata', ztorso);
% 
% %FEET
%     % draw the robot's body
%     Foot_length=0.2;
%     Foot_width=0.1;
%     SEPARACION_R=-0.113;
%     SEPARACION_L=0.113;
%     Right_Foot_plot = patch([Foot_length/2 -Foot_length/2 -Foot_length/2 Foot_length/2], [Foot_width/2+SEPARACION_R Foot_width/2+SEPARACION_R -Foot_width/2+SEPARACION_R -Foot_width/2+SEPARACION_R], [0 0 0 0], ...
%     'FaceColor', 'k', 'FaceAlpha', 0.5);
%     Left_Foot_plot = patch([Foot_length/2 -Foot_length/2 -Foot_length/2 Foot_length/2], [Foot_width/2+SEPARACION_L Foot_width/2+SEPARACION_L -Foot_width/2+SEPARACION_L -Foot_width/2+SEPARACION_L], [0 0 0 0], ...
%     'FaceColor', 'k', 'FaceAlpha', 0.5);

    
%WORLD COORDINATES
    world_coord_length=0.4;
    worldX=line('color','red', 'LineWidth', 2);
    set(worldX,'xdata', [0 world_coord_length], 'ydata', [0 0], 'zdata', [0 0]);
    worldY=line('color','green', 'LineWidth', 2);
    set(worldY,'xdata', [0 0], 'ydata', [0 world_coord_length], 'zdata', [0 0]);
    worldZ=line('color','blue', 'LineWidth', 2);
    set(worldZ,'xdata', [0 0], 'ydata', [0 0], 'zdata', [0 world_coord_length]);
    
    % cones of the axes
    [xc,yc,zc] = cylinder([0 world_coord_length/15]);
    zc(zc==0) = world_coord_length+world_coord_length/15;
    zc(zc==1) = world_coord_length-world_coord_length/15;
    worldX_cone=surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
    worldY_cone=surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
    worldZ_cone=surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');
    
hold off;

% walk!
% k = 1;
% while 1
%     legs(1).plot( gait(qcycle, k, 0,   0), plotopt);
%     legs(2).plot( gait(qcycle, k, 100, 0), plotopt);
%     legs(3).plot( gait(qcycle, k, 200, 1), plotopt);
%     legs(4).plot( gait(qcycle, k, 300, 1), plotopt);
%     drawnow
%     k = k+1;
% end


% RIGHT FOOT SUPPORT MATRIXES
HT_Matrix_Plot_Right_Leg(:,:,1) =  [0 -1 0 0;...
                        0 0 1 -2*TEO.legs.link_lengths(1);...
                        -1 0 0 0;...
                        0 0 0 1];

HT_Matrix_Plot_Right_Leg(:,:,2) =  [0 -1 0 0;...
                        0 0 1 0;...
                        -1 0 0 0;...
                        0 0 0 1];

                    
HT_Matrix_Plot_Torso(:,:,1) =  [1 0 0 0;...
                        0 1 0 -TEO.legs.link_lengths(1);...
                        0 0 1 0;...
                        0 0 0 1];

HT_Matrix_Plot_Torso(:,:,2) =  [1 0 0 0;...
                        0 1 0 TEO.legs.link_lengths(1);...
                        0 0 1 0;...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Leg(:,:,1) =  [0 -1 0 0;...
                        0 0 1 0;...
                        -1 0 0 0;...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Leg(:,:,2) =  [0 -1 0 0;...
                        0 0 1 2*TEO.legs.link_lengths(1);...
                        -1 0 0 0;...
                        0 0 0 1];

% HT_Matrix_Plot_Right_Arm =  [1 0 0 0;...
%                         0 -1 0 -TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
%                         0 0 -1 TEO.chest.link_lengths(2);...
%                         0 0 0 1];
                    
HT_Matrix_Plot_Right_Arm(:,:,1) =  [0 1 0 0;...
                        1 0 0 -TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];

HT_Matrix_Plot_Right_Arm(:,:,2) =  [0 1 0 0;...
                        1 0 0 TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Arm(:,:,1) =  [0 1 0 0;...
                        1 0 0 -TEO.legs.link_lengths(1)+TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Arm(:,:,2) =  [0 1 0 0;...
                        1 0 0 TEO.legs.link_lengths(1)+TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
                    
% Stop if there is not the 'q' variable
if ~exist('q')
    disp('ERROR!: There is not a "q" variable') 
    return
end;


waist_homogeneous_transform=zeros(4,4,size(q,2));
torso_homogeneous_transform=zeros(4,4,size(q,2));
CoM_homogeneous_transform=zeros(4,4,size(q,2));

waist_homogeneous_transform_left=zeros(4,4,size(q,2));

TEO_right_leg_base=zeros(4,4,size(q,2));
TEO_left_leg_base=zeros(4,4,size(q,2));
TEO_right_arm_base=zeros(4,4,size(q,2));
TEO_left_arm_base=zeros(4,4,size(q,2));
TEO_torso_base=zeros(4,4,size(q,2));

%Calculate transformations before movement
for k=1:size(q,2)
    
    if trajectory.SF(k)==0
        %Waist pose
        waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k)));
        waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
        waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));
        
        %Waist LEFT
        waist_pose_RPY_left= pose_quat2rpy(h.LF_T_w(q(:,k)));
        waist_rotation_left = RPY2Rot_Mat(waist_pose_RPY_left(4:6));
        waist_homogeneous_transform_left(:,:,k) = rt2tr(waist_rotation_left, waist_pose_RPY_left(1:3));   
        
        %Obtain Deduct CoM difference between RF_T_w and RL_T_w (only at
        %the beginning)
        if k==1
            waist_pose_RPY_difference=pose_quat2rpy(h.w_T_LF(q(:,k)))+waist_pose_RPY_left;
            waist_pose_RPY_difference(4:6)=0;
        end
        
        waist_rotation_difference = RPY2Rot_Mat(waist_pose_RPY_difference(4:6));
        waist_homogeneous_transform_difference(:,:,k) = rt2tr(waist_rotation_difference, waist_pose_RPY_difference(1:3));       

        %Torso pose
        torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);
        
        %Arms pose
        CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
        CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
        CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3)); 
        
        %Select legs,arms and torso bases
        TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
        %TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);
        TEO_left_leg_base(:,:,k) = waist_homogeneous_transform_difference(:,:,k)*waist_homogeneous_transform_left(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,2);

        TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
        TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);

        TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);
        
    
    elseif (trajectory.SF(k)==-1)
        %Waist pose
        waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k)));
        waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
        waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));

        %Torso pose
        torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);

        %Arms pose
        CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
        CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
        CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3)); 
        
        %Select legs,arms and torso bases
        TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
        TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);

        TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
        TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);

        TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);
        
    elseif trajectory.SF(k)==1
        %Waist pose
        waist_pose_RPY= pose_quat2rpy(h.LF_T_w(q(:,k)));
        waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
        waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));

        %Torso pose
        torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);

        %Arms pose
        CoM_pose_RPY= pose_quat2rpy(h.LF_T_CoM(q(:,k)));
        CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
        CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3));    

        TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,2);
        TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,2);

        TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,2);
        TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,2);

        TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,2);        
    else
        disp('ERROR!: Wrong trajectory Foot option') 
    end
        
        
end
    

for k=1:size(q,2)
    %Plot the number of q
    if exist('q_text')
        delete(q_text);
    end;
    
    q_text=text(0,0.5,1.5,strcat('qt=',num2str(k)),'Color','g');
    
    %Plot the support phase
    
    
    TEO_right_leg.base = TEO_right_leg_base(:,:,k);
    TEO_left_leg.base = TEO_left_leg_base(:,:,k);

    TEO_right_arm.base = TEO_right_arm_base(:,:,k);
    TEO_left_arm.base = TEO_left_arm_base(:,:,k);

    TEO_torso.base = TEO_torso_base(:,:,k);
    
    %%%Plot
    %Legs
    TEO_right_leg.plot(q(1:6,k)'); 
    TEO_left_leg.plot(q(7:12,k)');   
    
    %Torso
    TEO_torso.plot(q(13:14,k)'); 
    
    %Arms
    TEO_right_arm.plot(q(15:20,k)');  
    TEO_left_arm.plot(q(21:26,k)'); 
    
    drawnow
    
end