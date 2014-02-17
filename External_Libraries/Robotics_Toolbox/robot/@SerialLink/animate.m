%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot().
%
% Updates graphical instances of this robot in all figures.
%
% Notes::
% - Not a general purpose method, used for Simulink robot animation.
%
% See also SerialLink.plot.

function animate(robot, q, handles)

    if nargin < 3
        handles = findobj('Tag', robot.name);
    end

    for handle=handles'     % for each graphical robot instance
        h = get(handle, 'UserData');
        animate2( h, q);
    end


function animate2(h, q)

    robot = h.robot;
    n = robot.n;
    L = robot.links;
    humanoid_structure = robot.humanoid_structure;
    
    mag = h.mag;

    % compute the link transforms, and record the origin of each frame
    % for the animation.
    t = robot.base;
    Tn = t;
    x = zeros(1,n);
    y = zeros(1,n);
    z = zeros(1,n);
    xs = zeros(1,n);
    ys = zeros(1,n);
    zs = zeros(1,n);

    % add first point to the link/shadow line, the base
    x(1) = t(1,4);
    y(1) = t(2,4);
    z(1) = t(3,4);
    xs(1) = t(1,4);
    ys(1) = t(2,4);
    zs(1) = h.zmin;
    
    % add subsequent points
    for j=1:n
        Tn(:,:,j) = t;
        t = t * L(j).A(q(j));
        x(j+1) = t(1,4);
        y(j+1) = t(2,4);
        z(j+1) = t(3,4);
        xs(j+1) = t(1,4);
        ys(j+1) = t(2,4);
        zs(j+1) = h.zmin;
    end
    t = t *robot.tool;

    
        
  
    
    
    %
    % draw the robot stick figure and the shadow
    %
    
    set(h.links,'xdata', x, 'ydata', y, 'zdata', z);
    
    if isfield(h, 'shadow')
        set(h.shadow,'xdata', xs, 'ydata', ys, 'zdata', zs);
    end


    % Plot TEO's extensions
    
    if strcmp(robot.name, 'TEO_T')
        end_link_start = Tn(:,:,end);
        end_link_end = Tn(:,:,end)*transl(0,-humanoid_structure.torso.link_lengths(2),0);
        end_chest = end_link_end*transl(0,-humanoid_structure.chest.link_lengths(2),0);
        end_clavicle_right = end_chest*transl(0,0,-humanoid_structure.arms.link_lengths(1));
        end_clavicle_left = end_chest*transl(0,0,humanoid_structure.arms.link_lengths(1));
        set(h.end_link,'xdata',[end_link_start(1,4) end_link_end(1,4)], 'ydata', [end_link_start(2,4) end_link_end(2,4)], ...
            'zdata', [end_link_start(3,4) end_link_end(3,4)])
        set(h.chest,'xdata',[end_link_end(1,4) end_chest(1,4)], 'ydata', [end_link_end(2,4) end_chest(2,4)], ...
            'zdata', [end_link_end(3,4) end_chest(3,4)])            
        set(h.clavicle_right,'xdata',[end_chest(1,4) end_clavicle_right(1,4)], 'ydata', [end_chest(2,4) end_clavicle_right(2,4)], ...
            'zdata', [end_chest(3,4) end_clavicle_right(3,4)]) 
        set(h.clavicle_left,'xdata',[end_chest(1,4) end_clavicle_left(1,4)], 'ydata', [end_chest(2,4) end_clavicle_left(2,4)], ...
            'zdata', [end_chest(3,4) end_clavicle_left(3,4)]) 
        
       % Plot CoM 
       [xCoM, yCoM, zCoM] = sphere_coord( 0.03, end_link_end(1,4), end_link_end(2,4), end_link_end(3,4));
       
       set(h.CoM,'xdata',xCoM, 'ydata', yCoM, 'zdata', zCoM, 'FaceColor', 'm', 'EdgeColor', 'none')
       
    end
    
    if strcmp(robot.name, 'TEO_LR')
        waist_start = transl(0,humanoid_structure.legs.link_lengths(1),0)*robot.base;
        waist_end = transl(0,0,humanoid_structure.waist.link_lengths(2))*waist_start;
        set(h.waist,'xdata',[waist_start(1,4) waist_end(1,4)], 'ydata', [waist_start(2,4) waist_end(2,4)], ...
            'zdata', [waist_start(3,4) waist_end(3,4)])
        set(h.hip,'xdata',[robot.base(1,4) waist_start(1,4)], 'ydata', [robot.base(2,4) waist_start(2,4)], ...
            'zdata', [robot.base(3,4) waist_start(3,4)])  
        
        foot_front_right=transl(humanoid_structure.legs.right.foot.limits.x(2),humanoid_structure.legs.right.foot.limits.y(1),0)*t;
        foot_front_left=transl(humanoid_structure.legs.right.foot.limits.x(2),humanoid_structure.legs.left.foot.limits.y(2),0)*t;
        foot_back_right=transl(humanoid_structure.legs.right.foot.limits.x(1),humanoid_structure.legs.right.foot.limits.y(1),0)*t;
        foot_back_left=transl(humanoid_structure.legs.right.foot.limits.x(1),humanoid_structure.legs.right.foot.limits.y(2),0)*t;
        set(h.foot,'xdata',[foot_front_right(1,4) foot_front_left(1,4) foot_back_left(1,4) foot_back_right(1,4)],...
            'ydata',[foot_front_right(2,4) foot_front_left(2,4) foot_back_left(2,4) foot_back_right(2,4)], 'zdata',[foot_front_right(3,4) foot_front_left(3,4) foot_back_left(3,4) foot_back_right(3,4)])
    end
    
    if strcmp(robot.name, 'TEO_LL')
        hip_end = transl(0,-humanoid_structure.legs.link_lengths(1),0)*robot.base;
        set(h.hip,'xdata',[robot.base(1,4) hip_end(1,4)], 'ydata', [robot.base(2,4) hip_end(2,4)], ...
            'zdata', [robot.base(3,4) hip_end(3,4)]) 
        
        foot_front_right=transl(humanoid_structure.legs.left.foot.limits.x(2),humanoid_structure.legs.left.foot.limits.y(1),0)*t;
        foot_front_left=transl(humanoid_structure.legs.left.foot.limits.x(2),humanoid_structure.legs.left.foot.limits.y(2),0)*t;
        foot_back_right=transl(humanoid_structure.legs.left.foot.limits.x(1),humanoid_structure.legs.left.foot.limits.y(1),0)*t;
        foot_back_left=transl(humanoid_structure.legs.left.foot.limits.x(1),humanoid_structure.legs.left.foot.limits.y(2),0)*t;
        set(h.foot,'xdata',[foot_front_right(1,4) foot_front_left(1,4) foot_back_left(1,4) foot_back_right(1,4)],...
            'ydata',[foot_front_right(2,4) foot_front_left(2,4) foot_back_left(2,4) foot_back_right(2,4)], 'zdata',[foot_front_right(3,4) foot_front_left(3,4) foot_back_left(3,4) foot_back_right(3,4)])
    end
    
    %
    % display the joints as cylinders with rotation axes
    %
    if isfield(h, 'joint')
        xyz_line = [0 0; 0 0; -2*mag 2*mag; 1 1];

        for j=1:n
            % get coordinate data from the cylinder
            xyz = get(h.joint(j), 'UserData');
            xyz = Tn(:,:,j) * xyz;
            ncols = numcols(xyz)/2;
            xc = reshape(xyz(1,:), 2, ncols);
            yc = reshape(xyz(2,:), 2, ncols);
            zc = reshape(xyz(3,:), 2, ncols);

            set(h.joint(j), 'Xdata', xc, 'Ydata', yc, ...
                'Zdata', zc);

            xyzl = Tn(:,:,j) * xyz_line;
            if isfield(h, 'jointaxis')
                set(h.jointaxis(j), 'Xdata', xyzl(1,:), ...
                    'Ydata', xyzl(2,:), ...
                    'Zdata', xyzl(3,:));
                set(h.jointlabel(j), 'Position', xyzl(1:3,1));
            end
        end
    end

    %
    % display the wrist axes and labels
    %
    if isfield(h, 'x')
%         global prueba
%         prueba=Tn;
%         disp('graficando')
%         xv = Tn(:,:,1)*[mag;0;0;1];
%         yv = Tn(:,:,1)*[0;mag;0;1];
%         zv = Tn(:,:,1)*[0;0;mag;1];
%         set(h.x,'xdata',[Tn(1,4,1) xv(1)], 'ydata', [Tn(2,4,1) xv(2)], ...
%             'zdata', [Tn(3,4,1) xv(3)]);
%         set(h.y,'xdata',[Tn(1,4,1) yv(1)], 'ydata', [Tn(2,4,1) yv(2)], ...
%             'zdata', [Tn(3,4,1) yv(3)]);
%         set(h.z,'xdata',[Tn(1,4,1) zv(1)], 'ydata', [Tn(2,4,1) zv(2)], ...
%             'zdata', [Tn(3,4,1) zv(3)]);

       %%% DOMINGOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
       
        for j=1:n,
              
            xv_others = Tn(:,:,j)*[1.5*mag;0;0;1];
            yv_others = Tn(:,:,j)*[0;1.5*mag;0;1];
            zv_others = Tn(:,:,j)*[0;0;1.5*mag;1];
            set(h.x_others(j),'xdata',[Tn(1,4,j) xv_others(1)], 'ydata', [Tn(2,4,j) xv_others(2)], ...
                'zdata', [Tn(3,4,j) xv_others(3)]);
            set(h.y_others(j),'xdata',[Tn(1,4,j) yv_others(1)], 'ydata', [Tn(2,4,j) yv_others(2)], ...
                 'zdata', [Tn(3,4,j) yv_others(3)]);
            set(h.z_others(j),'xdata',[Tn(1,4,j) zv_others(1)], 'ydata', [Tn(2,4,j) zv_others(2)], ...
                'zdata', [Tn(3,4,j) zv_others(3)]);
            
            set(h.x_otherst(j), 'Position', xv_others(1:3));
            set(h.y_otherst(j), 'Position', yv_others(1:3));
            set(h.z_otherst(j), 'Position', zv_others(1:3));
            
        end;


        
        %
        % compute the wrist axes, based on final link transformation
        % plus the tool transformation.
        %
        xv = t*[1.5*mag;0;0;1];
        yv = t*[0;1.5*mag;0;1];
        zv = t*[0;0;1.5*mag;1];

        %
        % update the line segments, wrist axis and links
        %
        set(h.x,'xdata',[t(1,4) xv(1)], 'ydata', [t(2,4) xv(2)], ...
            'zdata', [t(3,4) xv(3)],'LineWidth',2);
        set(h.y,'xdata',[t(1,4) yv(1)], 'ydata', [t(2,4) yv(2)], ...
             'zdata', [t(3,4) yv(3)],'LineWidth',2);
        set(h.z,'xdata',[t(1,4) zv(1)], 'ydata', [t(2,4) zv(2)], ...
             'zdata', [t(3,4) zv(3)],'LineWidth',2);
        set(h.xt, 'Position', xv(1:3),'color', 'red');
        set(h.yt, 'Position', yv(1:3),'color', 'green');
        set(h.zt, 'Position', zv(1:3),'color', 'blue');
        
        

        
    end
    
