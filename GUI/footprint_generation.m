function varargout = footprint_generation(varargin)
% FOOTPRINT_GENERATION MATLAB code for footprint_generation.fig
%      FOOTPRINT_GENERATION, by itself, creates a new FOOTPRINT_GENERATION or raises the existing
%      singleton*.
%
%      H = FOOTPRINT_GENERATION returns the handle to a new FOOTPRINT_GENERATION or the handle to
%      the existing singleton*.
%
%      FOOTPRINT_GENERATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FOOTPRINT_GENERATION.M with the given input arguments.
%
%      FOOTPRINT_GENERATION('Property','Value',...) creates a new FOOTPRINT_GENERATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before footprint_generation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to footprint_generation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help footprint_generation

% Last Modified by GUIDE v2.5 20-Jan-2014 16:39:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @footprint_generation_OpeningFcn, ...
                   'gui_OutputFcn',  @footprint_generation_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before footprint_generation is made visible.
function footprint_generation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to footprint_generation (see VARARGIN)

clear P p Points
global init_floating_foot finish_both_feet
global nsteps step_length step_height Ts t_step ds_percentage 

global P p 
global g zc lambda beta alpha % Default Cog parameters

global humanoid_fields

P = [];
p = [];
step_length = 0.10;
nsteps = 0;
Ts = 0.02;
t_step = 4;
step_height = 0.10;

init_floating_foot = 'Right';
finish_both_feet = 1;
ds_percentage = 20;

g = 9.1;
zc = 1.0;
lambda = 0.7;
alpha = 0.3;
beta = 0.3;


humanoid_fields = humanoid_operational_fields ();

% Choose default command line output for footprint_generation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Initial Floor Limits
set(handles.edit_xmin,'String',-1);
set(handles.edit_xmax,'String',2);
set(handles.edit_ymin,'String',-1)
set(handles.edit_ymax,'String',1)
set(handles.edit_xscale,'String',0.2)
set(handles.edit_yscale,'String',0.2)
xmin = str2double(get(handles.edit_xmin,'String'));
xmax = str2double(get(handles.edit_xmax,'String'));
ymin = str2double(get(handles.edit_ymin,'String'));
ymax = str2double(get(handles.edit_ymax,'String'));
axis(handles.axes_flat_floor, [xmin xmax ymin ymax]);

% Panel visualization
grid(handles.axes_flat_floor,'on');
xscale = str2double(get(handles.edit_xscale,'String'));
yscale = str2double(get(handles.edit_yscale,'String'));
set(handles.axes_flat_floor,'XTick',xmin:xscale:xmax);
set(handles.axes_flat_floor,'YTick',ymin:yscale:ymax);

% Plot world Frame
plot_world_frame(handles)

% Plot footprints
posfootprintRF = [0, -0.1186, 0];
posfootprintLF = [0, 0.1186, 0];
% plot(plot::Circle2d(0.1, [0, -0.1186]), plot::Circle2d(0.2, [0, 0.1186]));
% radius = 0.03;
% footprintRF = plot2dcircle(0, -0.1186, radius);
% footprintLF = plot2dcircle(0, 0.1186, radius);

plotfootprint(posfootprintRF,0);
plotfootprint(posfootprintLF,0);
% plotfootprint(posfootprintRF);
% plotfootprint(posfootprintLF);

set(gcf,'toolbar','figure');


function varargout = footprint_generation_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


function pushbutton_global_manual_Callback(hObject, eventdata, handles)
plane = handles.axes_flat_floor;
set(plane,'buttondownfcn',@start_manual_curve)


function start_manual_curve(src,eventdata)
% Tell window that we'll handle drag and up events
set(gcf,'keypressfcn',        @onkeypress);
set(gcf,'windowbuttonmotionfcn', @plotondrag);
set(gcf,'windowbuttonupfcn', @plotonup);
append_current_point(src);


function plotondrag(src,ev)
append_current_point(gca);


function plotonup(src,ev)
% Tell window to handle down, drag and up events itself
finish_plot();


function append_current_point(src)
global Points p thetas;
% get current mouse position
cp = get(src,'currentpoint');
% append to running list
Points = [Points; cp(1,:)];
if isempty(p)
  % init plot
  hold on;
  p = plot(Points(:,1),Points(:,2));
  hold off;
else
  % update plot
  set(p,'Xdata',Points(:,1),'Ydata',Points(:,2));
end 
thetas = atan(Points(:,1)/Points(:,2));

function finish_plot()
set(gcf,'windowbuttonmotionfcn','');
set(gcf,'windowbuttonupfcn','');
set(gcf,'windowbuttondownfcn','');
set(gcf,'keypressfcn','');

function onkeypress(src,ev)
% escape character id
ESC = char(27);
switch ev.Character
case ESC
  finish_plot();
% otherwise
%   error(['Unknown key: ' ev.Character]);
end

function pushbutton_global_line_Callback(hObject, eventdata, handles)
plane = handles.axes_flat_floor;
set(plane,'buttondownfcn',@start_pencil)

function start_pencil(src,eventdata)
coords=get(src,'currentpoint'); %since this is the axes callback, src=gca
x=coords(1,1,1);
y=coords(1,2,1);

r=line(x, y, 'color', [0 .5 1], 'LineWidth', 2, 'hittest', 'off'); %turning hittset off allows you to draw new lines that start on top of an existing line.
set(gcf,'windowbuttonmotionfcn',{@continue_pencil,r})
%set(gcf,'windowbuttonupfcn',@done_pencil)

function continue_pencil(src,eventdata,r)
%Note: src is now the figure handle, not the axes, so we need to use gca.
coords=get(gca,'currentpoint'); %this updates every time i move the mouse
x=coords(1,1,1);
y=coords(1,2,1);
%get the line's existing coordinates and append the new ones.
lastx=get(r,'xdata');  
lasty=get(r,'ydata');
newx=[lastx x];
newy=[lasty y];
set(r,'xdata',newx,'ydata',newy);

function done_pencil(src,evendata)
%all this funciton does is turn the motion function off 
set(gcf,'windowbuttonmotionfcn','')
set(gcf,'windowbuttonupfcn','')


function pushbutton_floor_options_Callback(hObject, eventdata, handles)
% Change the Floor (Panel) Limits and scale
xmin = str2double(get(handles.edit_xmin,'String'));
xmax = str2double(get(handles.edit_xmax,'String'));
ymin = str2double(get(handles.edit_ymin,'String'));
ymax = str2double(get(handles.edit_ymax,'String'));
axis(handles.axes_flat_floor, [xmin xmax ymin ymax]);
xscale = str2double(get(handles.edit_xscale,'String'));
yscale = str2double(get(handles.edit_yscale,'String'));
set(handles.axes_flat_floor,'XTick',xmin:xscale:xmax);
set(handles.axes_flat_floor,'YTick',ymin:yscale:ymax);


function edit_xmin_Callback(hObject, eventdata, handles)


function edit_xmin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_xmax_Callback(hObject, eventdata, handles)


function edit_xmax_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ymin_Callback(hObject, eventdata, handles)


function edit_ymin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_ymax_Callback(hObject, eventdata, handles)


function edit_ymax_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_xscale_Callback(hObject, eventdata, handles)


function edit_xscale_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_yscale_Callback(hObject, eventdata, handles)


function edit_yscale_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_generate_nsteps_Callback(hObject, eventdata, handles)
global init_floating_foot median_zmp finish_both_feet
global footprints_poses
global step_length
nsteps = str2double(get(handles.edit_nsteps,'String'));

step_length = str2double(get(handles.edit_step_length,'String'));
if isnan(step_length)
    error('WARNING: Step Length cant be zero. Default step_length=0.1');
    step_length = 0.1;
end

if finish_both_feet
    nfootprints = nsteps+3;
else
    nfootprints = nsteps+2;
end

footprints_poses = zeros(6,nfootprints);

posfootprintRF = [0; -0.1186; 0];
posfootprintLF = [0; 0.1186; 0];

if strcmp(init_floating_foot,'Right')
    startLF = 1;
else
    startLF = 0;
end

thetaincr = str2double(get(handles.edit_theta_variation,'String'));
if isnan(thetaincr)
    thetaincr = 0;
end

% First two foot prints
if startLF==0,
    footprints_poses(1:3,1) = posfootprintLF;
    footprints_poses(1:3,2) = posfootprintRF;
else
    footprints_poses(1:3,1) = posfootprintRF;
    footprints_poses(1:3,2) = posfootprintLF;
end

% N footprints
for ii = 1:nsteps,
    poscurrentfootprint = footprints_poses(1:3,ii+1);
    if ii==1
%         xvar = step_length/4; % El primer paso solo sera un paso completo de un paso completo
          xvar = step_length/2;
    else
        xvar = step_length/2;
    end
    yvar = 0.1186*2;
    zvar = 0;
    theta = footprints_poses(6,ii+1) + thetaincr;
    footprints_poses(1:3,ii+2) = poscurrentfootprint + rotz(theta)*[xvar; -(-1)^(ii+startLF)*yvar; zvar];
    footprints_poses(6,ii+2) = theta;
end

% Last footprint (only if user selects finish with both feet)
if finish_both_feet
    poscurrentfootprint = footprints_poses(1:3,end-1);
    xvar = 0;
    theta = footprints_poses(6,end-1); % Same orientation than last footprint
    footprints_poses(1:3,end) = poscurrentfootprint + rotz(theta)*[xvar; -(-1)^(ii+startLF+1)*yvar; zvar];
    footprints_poses(6,end) = theta;
end

% Plot all footprints (except first two)
for jj=3:size(footprints_poses,2)
    plotfootprint(footprints_poses(1:3,jj),footprints_poses(6,jj));
end

median_zmp = calculate_median_zmp (footprints_poses);
plot_median_zmp (median_zmp)

function median_zmp = calculate_median_zmp (footprints)
median_zmp = zeros(2,size(footprints,2)-1);

for n=1:size(median_zmp,2)
    median_zmp(:,n) = (footprints(1:2,n)+footprints(1:2,n+1))/2;
end
    

function edit_nsteps_Callback(hObject, eventdata, handles)


function edit_nsteps_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_theta_variation_Callback(hObject, eventdata, handles)


function edit_theta_variation_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_init_sf_Callback(hObject, eventdata, handles)
global init_floating_foot
contents = cellstr(get(hObject,'String'));
init_floating_foot = contents{get(hObject,'Value')};


function popupmenu_init_sf_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_local_motion_feet_Callback(hObject, eventdata, handles)


function popupmenu_local_motion_feet_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_reset_nsteps_Callback(hObject, eventdata, handles)
global footprints footprintstheta median_zmp
global trajectory d_trajectory dd_trajectory
cla (handles.axes_flat_floor)
plot_world_frame(handles)
% Plot footprints
posfootprintRF = [0, -0.1186, 0];
posfootprintLF = [0, 0.1186, 0];

plotfootprint(posfootprintRF,0);
plotfootprint(posfootprintLF,0);
footprints = [];
footprintstheta = [];
median_zmp = [];
trajectory = [];
d_trajectory = [];
dd_trajectory = [];


function pushbutton_local_motion_feet_Callback(hObject, eventdata, handles)
global init_floating_foot footprints_poses
global Ts t_step step_height
global trajectory d_trajectory dd_trajectory
global ds_percentage

ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'));
if isnan(ds_percentage)
    ds_percentage = 20;
end

ds_time = round_to_Ts(ds_percentage/100*t_step, Ts);
ss_time = round_to_Ts(t_step - ds_time, Ts);
current_time = 0;

%DEBUG
global pp_FF dpp_FF ddpp_FF

% RF_poses
% LF_poses
[ trajectory d_trajectory dd_trajectory ] = create_TEO_structures( Ts );
humanoid_fields = humanoid_operational_fields ();

floating_foot = init_floating_foot;

% for ii=1:size(footprints_poses,2)-1
for ii=2:size(footprints_poses,2)-1 %DEBE SER -1, HAY QUE AGREGAR LA HUELLA DE DOBLE SOPORTE
    % Double Support Phase 1 (half of Double Support phase)
    initial_time = current_time;
    final_time = initial_time + ds_time/2;
    
    if ii==2,
        if strcmp(floating_foot,'Left')
            pLF = footprints_poses(:,ii);
            pRF = footprints_poses(:,ii-1);
        else
            pLF = footprints_poses(:,ii);
            pRF = footprints_poses(:,ii-1);
        end

        % Left Foot
        P1_LF = set_trajectory_condition(initial_time, pLF, zeros(6,1), zeros(6,1));
        P2_LF = set_trajectory_condition(final_time, pLF, zeros(6,1), zeros(6,1));
        P_LF = [P1_LF P2_LF];
        [pp_LF, dpp_LF, ddpp_LF] = poly5_viapoints_trajectory (P_LF, Ts);
        trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_LF,  'LF');
        d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_LF, 'LF');
        dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_LF, 'LF');

        % Right Foot
        P1_RF = set_trajectory_condition(initial_time, pRF, zeros(6,1), zeros(6,1));
        P2_RF = set_trajectory_condition(final_time, pRF, zeros(6,1), zeros(6,1));
        P_RF = [P1_RF P2_RF];
        [pp_RF, dpp_RF, ddpp_RF] = poly5_viapoints_trajectory (P_RF, Ts);
        trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_RF,  'RF');
        d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_RF, 'RF');
        dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_RF, 'RF');
        
        %DEBUG
        datads1 = size(pp_RF.data);
        Tds1 = pp_RF.T;
        
    
    else
        SF = 0;
        time_DS = initial_time:final_time;
        pp_SF  = create_trajectory_structure(SF*ones(size(time_DS)), Ts, time_DS);        
        trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
        d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF, 'SF');
        dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF, 'SF'); 
                
        %DEBUG
        datads1 = size(pp_RF.data);
        Tds1 = pp_RF.T;
    end
        
    current_time = final_time;
    
    % Single Support Phase
    p0_ff = footprints_poses(:,ii-1);
    p2_ff = footprints_poses(:,ii+1);
    
    p1_ff = (p0_ff+p2_ff)/2 + [0;0;step_height;0;0;0];
    initial_time = current_time;
    climbing_time = initial_time + ss_time/2;
    landing_time = climbing_time + ss_time/2;
    
    P0 = set_trajectory_condition(initial_time, p0_ff, zeros(6,1), zeros(6,1));
    P1 = set_trajectory_condition(climbing_time, p1_ff, zeros(6,1), zeros(6,1));
    P2 = set_trajectory_condition(landing_time, p2_ff, zeros(6,1), zeros(6,1));
        
    P = [P0 P1 P2];
    [pp_FF, dpp_FF, ddpp_FF] = poly5_viapoints_trajectory (P, Ts);
       
    if strcmp(floating_foot,'Left')
        trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_FF,  'LF');
        d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_FF, 'LF');
        dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_FF, 'LF');
        SF = 1;
        floating_foot = 'Right';
    else
        trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_FF,  'RF');
        d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_FF, 'RF');
        dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_FF, 'RF');
        SF = -1;
        floating_foot = 'Left';
    end
    
    pp_SF  = create_trajectory_structure(SF*ones(size(pp_FF.time)), Ts, pp_FF.time);    
    trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
    d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF, 'SF');
    dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF, 'SF');
    
%     % Double Support Phase 2 (half of Double Support phase)
    SF = 0;
    double_supp_time = landing_time + ds_time/2;
    time_DS = landing_time:Ts:double_supp_time;

    pp_SF  = create_trajectory_structure(SF*ones(size(time_DS)), Ts, time_DS);
    trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
    d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF, 'SF');
    dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF, 'SF');
    
    current_time = double_supp_time;
end

% Plot Feet trajectories
plot_feet_trajectories();


function pushbutton_local_motion_cart_table_Callback(hObject, eventdata, handles)
% global trajectory d_trajectory dd_trajectory
% global zc
% global footprints_SF footprints_poses
% global median_zmp t_step Ts
% current_time = 0;
% 
% global zmp_trajectory_CartTable
% 
% zmp_points = zeros(2,size(footprints_poses,2)-1);
% zmp_trajectory_CartTable = zeros(2,size(trajectory.CoM,2));
% 
% zmp_points(:,1) = median_zmp(:,1);
% 
% for ii=2:size(footprints_poses,2)
%     p1_zmp = median_zmp(1:2,ii-2);
%     p2_zmp = footprints_poses(1:2,ii-1);
%     p3_zmp = median_zmp(1:2,ii-1);
%    
%     init_time = current_time;
%     end_time = init_time + t_step/2;
%     
%     DM=[p1_zmp;p2_zmp];
%     
%     current_time = ff_time + t_step/2;
%     
% %     zmp_trajectory(1,:) = p1_zmp(1):1/Ts:p2_zmp(1);
% %     zmp_trajectory(2,:) = p1_zmp(2):1/Ts:p2_zmp(2);
% end
% 
% hold on
% plot3(zmp_trajectory(1,:), zmp_trajectory(2,:), zeros(1,size(zmp_trajectory,2)), 'color', 'm');
% hold off

global footprints_poses finish_both_feet median_zmp
global t_step Ts ds_percentage 
global zmp_trajectory
current_time = 0;
ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'));
if isnan(ds_percentage)
    ds_percentage = 20;
end

% SI NO TERMINA EN DS DEBERA TENER UNA COLUMNA MENOS
zmp_points = zeros(2,2*size(footprints_poses,2));


ds_time = round_to_Ts(ds_percentage/100*t_step, Ts);
ss_time = round_to_Ts(t_step - ds_time, Ts);


global P
zmp_change = 0.1;
for ii=2:size(footprints_poses,2)-1
    if ii==2
        P = set_trajectory_condition(current_time, [median_zmp(1:2,ii-1);0], zeros(3,1), zeros(3,1));
    end
current_time = current_time+ds_time/2;
P1 = set_trajectory_condition(current_time, [footprints_poses(1:2,ii);0], zeros(3,1), zeros(3,1));
current_time = current_time+ss_time;
P2 = set_trajectory_condition(current_time, [footprints_poses(1:2,ii);0], zeros(3,1), zeros(3,1));
current_time = current_time+ds_time/2;
P3 = set_trajectory_condition(current_time, [median_zmp(1:2,ii);0], zeros(3,1), zeros(3,1));
P  = [P P1 P2 P3];
end
[zmp_trajectory, dx, ddx] = poly5_viapoints_trajectory (P, Ts);
hold on
plot3(zmp_trajectory.data(1,:), zmp_trajectory.data(2,:), zmp_trajectory.data(3,:), 'color', 'm');
hold off


function pushbutton_local_motion_cog_Callback(hObject, eventdata, handles)
global trajectory d_trajectory dd_trajectory
global g zc lambda beta alpha
global footprints_SF footprints_poses median_zmp
global t_step Ts
global humanoid_fields
global ds_percentage

if ~isnan(str2double(get(handles.edit_cog_zc,'String')))
    zc = str2double(get(handles.edit_cog_zc,'String'));
end

if ~isnan(str2double(get(handles.edit_cog_lambda,'String')))
    lambda = str2double(get(handles.edit_cog_lambda,'String'));
end

if ~isnan(str2double(get(handles.edit_cog_beta,'String')))
    beta = str2double(get(handles.edit_cog_beta,'String'));
end

if ~isnan(str2double(get(handles.edit_cog_alpha,'String')))
    alpha = str2double(get(handles.edit_cog_alpha,'String'));
end


% Calculate ds_time and ss_time
t0 = 0;
ds_time = round_to_Ts(ds_percentage/100*t_step, Ts);
ss_time = round_to_Ts(t_step - ds_time, Ts);

% Select the model to generate the COG trajectory
cog_list_options = cellstr(get(handles.popupmenu_local_motion_cog,'String'));
cog_option = cog_list_options{get(handles.popupmenu_local_motion_cog,'Value')};

switch cog_option
    case 'Polynomial 5'
        [cog_traj, dcog_traj, ddcog_traj] = polynomial5_trajectory(footprints_poses, median_zmp, Ts,  ss_time, ds_time, t0, zc);
%         tamano_cog_poly5 = size(cog_traj.data)
%         tamano_dcog_poly5 = size(dcog_traj.data)
%         tamano_ddcog_poly5 = size(ddcog_traj.data)
    case '3D LIPM'
        [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, Ts, ss_time, ds_time, t0, zc, beta, lambda, g);
%         tamano_cog_3dlipm = size(cog_traj.data)
%         tamano_cog_3dlipm = size(dcog_traj.data)
%         tamano_cog_3dlipm = size(ddcog_traj.data)
    otherwise
        % - Cart Table
        % - Polynomial 3
        % - Polynomial 7
        disp('ERROR: Opcion no implementada');
        return;
end

trajectory   = insert_trajectory(trajectory,   humanoid_fields, cog_traj,  'CoM');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dcog_traj, 'CoM');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddcog_traj, 'CoM');

% Plot CoM trajectories
plot_CoM_trajectory();


function [cog_traj, dcog_traj, ddcog_traj] = polynomial5_trajectory(footprints_poses, median_zmp, Ts,  TimeSS, TimeDS, t0, zc)

current_time = t0;

for ii=2:size(footprints_poses,2)-1,
    TDS1 = current_time;
%     TSSprev = round_to_Ts(t0 + TimeDS/2,Ts);
%     TSS = round_to_Ts(TSSprev + TimeSS/2,Ts);
    TSS = round_to_Ts(current_time + TimeDS/2 + TimeSS/2,Ts);
%     TSSnext = round_to_Ts(TSS + TimeSS/2,Ts);
%     TDS2 = round_to_Ts(TSSnext + TimeDS/2,Ts);
    TDS2 = round_to_Ts(TSS + TimeSS/2 + TimeDS/2,Ts);

%     DSglobal_pose1 = [median_zmp(:,ii-1);zc;0;0;mean([footprints_poses(6,ii),footprints_poses(6,ii-1)])];
%     DSglobal_pose2 = [median_zmp(:,ii);zc;0;0;mean([footprints_poses(6,ii),footprints_poses(6,ii+1)])];
    
    p1_com = [median_zmp(:,ii-1);zc;0;0;mean([footprints_poses(6,ii),footprints_poses(6,ii-1)])];
    p5_com = [median_zmp(:,ii);zc;0;0;mean([footprints_poses(6,ii),footprints_poses(6,ii+1)])];
    
    p3_com = footprints_poses(:,ii); p3_com(3) = zc;
    
%     % Change global reference to foot reference    
%     DSpose1 = pose_tr2rpy(pose_rpy2tr(p3_com)\pose_rpy2tr(DSglobal_pose1));
%     DSpose2 = pose_tr2rpy(pose_rpy2tr(p3_com)\pose_rpy2tr(DSglobal_pose2));
%     
%     lambdapose1 = DSpose1(1)*lambda;
%     lambdapose2 = DSpose2(1)*lambda;
% 
%     betapose1 = DSpose1(2)*beta;
%     betapose2 = DSpose1(2)*beta;
%     
%     
%     p2_com = footprints_poses(:,ii-1); p2_com(3) = zc;
%     p4_com = footprints_poses(:,ii-1); p4_com(3) = zc;
% 
%     % Convert foot reference to global reference
%     p2_com = transform_frame(pose_rpy2tr(footprints_poses(:,ii)), p2_com);
%     p4_com = transform_frame(pose_rpy2tr(footprints_poses(:,ii)), p4_com);
    
    P1 = set_trajectory_condition(TDS1, p1_com, zeros(6,1), zeros(6,1));
%     P2 = set_trajectory_condition(TSSprev, p2_com, zeros(6,1), zeros(6,1));
    P3 = set_trajectory_condition(TSS, p3_com, zeros(6,1), zeros(6,1));
%     P4 = set_trajectory_condition(TSSnext, p4_com, zeros(6,1), zeros(6,1));
    P5 = set_trajectory_condition(TDS2, p5_com, zeros(6,1), zeros(6,1));
%     P = [P P1 P2 P3 P4 P5];

    if ii==2,
        P = [P1 P3 P5];
    else
        P = [P P3 P5];
    end
    current_time = TDS2;
    
end

[cog_traj, dcog_traj, ddcog_traj] = poly5_viapoints_trajectory (P, Ts);


function calculate_zmp_trajectory_from_cog(trajectory, d_trajectory, dd_trajectory)
global g zc
global zmp_trajectory
% zmp_trajectory = zeros(2,size(trajectory.CoM,2));

zmp_trajectory = trajectory.CoM(1:2,:)-(zc/g)*dd_trajectory.CoM(1:2,:);

hold on
plot3(zmp_trajectory(1,:), zmp_trajectory(2,:), zeros(1,size(zmp_trajectory,2)), 'color', 'm');
hold off


function checkbox_finish_both_feet_Callback(hObject, eventdata, handles)
global finish_both_feet
finish_both_feet = get(hObject,'Value');


function edit_step_length_Callback(hObject, eventdata, handles)


function edit_step_length_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_step_height_Callback(hObject, eventdata, handles)


function edit_step_height_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_time_step_Callback(hObject, eventdata, handles)


function edit_time_step_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_ts_Callback(hObject, eventdata, handles)


function edit_ts_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_double_support_percentage_Callback(hObject, eventdata, handles)


function edit_double_support_percentage_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton10_Callback(hObject, eventdata, handles)
global trajectory
 figure(500), plot(trajectory.time,trajectory.RF(1,:),'*'),hold on, plot(trajectory.time,trajectory.LF(1,:),'*r'),plot(trajectory.time,0.3*trajectory.SF,'-g'), hold off


function pushbutton11_Callback(hObject, eventdata, handles)
global trajectory
 figure(501), plot(trajectory.time,trajectory.RF(2,:),'*'),hold on, plot(trajectory.time,trajectory.LF(2,:),'*r'),plot(trajectory.time,0.3*trajectory.SF,'-g'), hold off


function pushbutton_cart_table_cog_Callback(hObject, eventdata, handles)
global zmp_trajectory


function edit_cog_lambda_Callback(hObject, eventdata, handles)


function edit_cog_lambda_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_alpha_Callback(hObject, eventdata, handles)


function edit_cog_alpha_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_beta_Callback(hObject, eventdata, handles)


function edit_cog_beta_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_zc_Callback(hObject, eventdata, handles)


function edit_cog_zc_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_g_Callback(hObject, eventdata, handles)


function edit_cog_g_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit26_Callback(hObject, eventdata, handles)


function edit26_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_local_motion_cog_Callback(hObject, eventdata, handles)


function popupmenu_local_motion_cog_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton14_Callback(hObject, eventdata, handles)
plot_CoM_trajectory();


function plot_CoM_trajectory()
% Plot CoM trajectory
global plotCoM
global trajectory

if (exist('trajectory','var') && isstruct(trajectory))
   if ~isempty(trajectory.CoM)
    if ishandle(plotCoM),
        delete(plotCoM);
    end

    hold on
    plotCoM = plot3(trajectory.CoM(1,:),trajectory.CoM(2,:),trajectory.CoM(3,:),'color','b');
    hold off
   else
    disp('PLOT_COM ERROR: There is not a CoM trajectory to plot');   
   end
else
    disp('PLOT_COM ERROR: There is not a trajectory structure to plot');
end


function plot_feet_trajectories()
% Plot Feet Trajectories
global plotRF_traj plotLF_traj
global trajectory

if (exist('trajectory','var') && isstruct(trajectory))
   if ~isempty(trajectory.RF)
    if ishandle(plotRF_traj),
        delete(plotRF_traj);
    end
    hold on
    plotRF_traj = plot3(trajectory.RF(1,:),trajectory.RF(2,:),trajectory.RF(3,:),'color','red');
    hold off
   else
    disp('PLOT_FEET ERROR: There is not a Right Foot trajectory to plot');   
   end
   if ~isempty(trajectory.LF)
    if ishandle(plotLF_traj),
        delete(plotLF_traj);
    end   
    hold on
    plotLF_traj = plot3(trajectory.LF(1,:),trajectory.LF(2,:),trajectory.LF(3,:),'color','red');
    hold off
   else
    disp('PLOT_FEET ERROR: There is not a Left Foot trajectory to plot');   
   end
else
    disp('PLOT_FEET ERROR: There is not a trajectory structure to plot');
end


function plot_median_zmp (median_zmp)
% Plot Median_ZMP trajectory
global plotMedianZMP

if ~isempty(median_zmp)
    if ishandle(plotMedianZMP),
        delete(plotMedianZMP);
    end

    for ii=1:size(median_zmp,2);
        hold on
        plotMedianZMP = plot(median_zmp(1,ii),median_zmp(2,ii),'X','linewidth',5,'color','m');
        hold off
    end
else
    disp('PLOT_MEDIAN_ZMP ERROR: There is not a CoM trajectory to plot');
end


function h = plotfootprint(posfootprint, theta)

footwidth = 0.15;
footlength = 0.25;
% footprintRF = rectangle('Position', [posfootprint(1)-footlength/2, posfootprint(2)-footwidth/2, footlength, footwidth]);
X = [-footlength/2 footlength/2 footlength/2 -footlength/2 -footlength/2];
Y = [footwidth/2 footwidth/2 -footwidth/2 -footwidth/2 footwidth/2];
P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
P = R * P;
hold on
    h = plot(P(1,:)+posfootprint(1), P(2,:)+posfootprint(2),'k');
hold off


function plot_world_frame(handles)

%En caso hubiera mas plots usar: axes(handles.axes1) para cambiar el gca

% WORLD COORDINATES
world_coord_length = 0.2;
worldX = line('color','red', 'LineWidth', 2);
set(worldX,'xdata', [0 world_coord_length], 'ydata', [0 0], 'zdata', [0 0]);
worldY = line('color','green', 'LineWidth', 2);
set(worldY,'xdata', [0 0], 'ydata', [0 world_coord_length], 'zdata', [0 0]);
worldZ = line('color','blue', 'LineWidth', 2);
set(worldZ,'xdata', [0 0], 'ydata', [0 0], 'zdata', [0 world_coord_length]);
    
% cones of the axes
[xc,yc,zc] = cylinder([0 world_coord_length/15]);
zc(zc==0) = world_coord_length + world_coord_length/15;
zc(zc==1) = world_coord_length - world_coord_length/15;
worldX_cone = surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
worldY_cone = surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
worldZ_cone = surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');


function plot2dcircle(x,y,r)
hold on
    th = 0:pi/50:2*pi;

    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;

    h = plot(xunit, yunit,'color','g');

hold off

% function pushbutton_3dlipm_cog_Callback(hObject, eventdata, handles)
% global trajectory d_trajectory dd_trajectory
% global footprints_poses median_zmp 
% global g Ts
% global ds_percentage t_step
% 
% t0 = 0
% 
% ds_time = round_to_Ts(ds_percentage/100*t_step, Ts);
% ss_time = round_to_Ts(t_step - ds_time, Ts);
% 
% zc = str2double(get(handles.edit_cog_zc,'String'));
% 
% lambda = str2double(get(handles.edit_cog_lambda,'String'));
% alpha = str2double(get(handles.edit_cog_alpha,'String'));
% beta = str2double(get(handles.edit_cog_beta,'String'));
% 
% humanoid_fields = humanoid_operational_fields ();
% 
% [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, Ts, ss_time, ds_time, t0, zc, beta, lambda, g);
% 
% trajectory   = insert_trajectory(trajectory,   humanoid_fields, cog_traj,  'CoM');
% d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dcog_traj, 'CoM');
% dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddcog_traj, 'CoM');
% 
% 
% % Plot CoM trajectories
% plot_CoM_trajectory();
