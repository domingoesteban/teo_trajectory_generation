function varargout = settings_trajectory_generation(varargin)
% ***************************
% SETTINGS_TRAJECTORY_GENERATION MATLAB code for settings_trajectory_generation.fig
% This window generates the main functions about Trajectory Generation
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @settings_trajectory_generation_OpeningFcn, ...
                   'gui_OutputFcn',  @settings_trajectory_generation_OutputFcn, ...
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


% --- Executes just before settings_trajectory_generation is made visible.
%                                           ---------  settings_trajectory_generation
function settings_trajectory_generation_OpeningFcn(hObject, eventdata, handles, varargin)
% -----------------
% Initial function
% -----------------
global SETTINGS_TEO

handles.output = hObject;
% -----------------------
scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);
% ------------------------
axes(handles.axes1)
[r,map]=imread('uc3m.png','png');
image(r);colormap(map);axis off
% ------------------------
SETTINGS_TEO.units.pos = 'm';
SETTINGS_TEO.units.orient = 'rad';
SETTINGS_TEO.units.def = '';

SETTINGS_TEO.parameters.Ts = 0;
SETTINGS_TEO.parameters.kp = 0;
SETTINGS_TEO.parameters.ko = 0;

SETTINGS_TEO.save.fdat = 0;
SETTINGS_TEO.save.fcsv = 0;

SETTINGS_TEO.q0.from = '';
SETTINGS_TEO.q0.data = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...                % Right Leg
     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...                     % Left Leg
     0.0349065850398866; 0;...                                                                                                      % Waist
     1.57079632679490; -0.167017153300893; 0; -0.734875474523928; 0; 0;                                                             % Right Arm
     1.57079632679490; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                                            % Left Arm ;

SETTINGS_TEO.body_parts.RH = 1;
SETTINGS_TEO.body_parts.LH = 1;
SETTINGS_TEO.body_parts.RF = 1;
SETTINGS_TEO.body_parts.LF = 1;
SETTINGS_TEO.body_parts.CoM = 1;
% ------------------------
% Update handles structure
guidata(hObject, handles);


function varargout = settings_trajectory_generation_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;



% *************************** MENUS ******************************
% --------------------------------------------------------------------
function main_menu_of_settings_Callback(hObject, eventdata, handles)
main_TEO
close(handles.figure1)
% *************************************************************************
%                                               ---- push_continue_Callback
function push_continue_Callback(hObject, eventdata, handles)

global SETTINGS_TEO manipulator_data trajectory d_trajectory dd_trajectory Trajectory d_Trajectory dd_Trajectory
SETTINGS_TEO.parameters.Ts = str2num(get(handles.Ts_edit,'String'));
SETTINGS_TEO.parameters.kp = str2num(get(handles.kp_edit,'String'));
SETTINGS_TEO.parameters.ko = str2num(get(handles.ko_edit,'String'));
CELDA_M = cell(4,2);
CELDA_M{2,1}= zeros(2,1); CELDA_M{2,2}=zeros(2,1);

CELDA_M{3,1}= {'' ''};CELDA_M{3,2}= {'' ''};

CELDA_M{4,1}= '';CELDA_M{4,2}= '';

manipulator_data.RA.datas = CELDA_M;
manipulator_data.RA.count_traj = zeros(1,2);
manipulator_data.LA.datas = CELDA_M;
manipulator_data.LA.count_traj = zeros(1,2);
manipulator_data.RLS.datas = CELDA_M;
manipulator_data.RLS.count_traj = zeros(1,2);
manipulator_data.RLF.datas = CELDA_M;
manipulator_data.RLF.count_traj = zeros(1,2);
manipulator_data.LLS.datas = CELDA_M;
manipulator_data.LLS.count_traj = zeros(1,2);
manipulator_data.LLF.datas = CELDA_M;
manipulator_data.LLF.count_traj = zeros(1,2);

% -------------------------------------------------------------------------
SETTINGS_TEO.h = TEO_kinematics_library;
if exist('TEO', 'var') == 0
    SETTINGS_TEO.TEO = TEO_structure('numeric', SETTINGS_TEO.units.orient, SETTINGS_TEO.units.pos); 
end
SETTINGS_TEO.humanoid_fields = humanoid_operational_fields (); 

trajectory = create_trajectory_template (SETTINGS_TEO.humanoid_fields, (SETTINGS_TEO.parameters.Ts));
d_trajectory = create_trajectory_template (SETTINGS_TEO.humanoid_fields, (SETTINGS_TEO.parameters.Ts));
dd_trajectory = create_trajectory_template (SETTINGS_TEO.humanoid_fields, (SETTINGS_TEO.parameters.Ts));
q0 = SETTINGS_TEO.q0.data;
Ts = SETTINGS_TEO.parameters.Ts;

% TEO is standing with both foot on the floor
pose_RF = [0; -SETTINGS_TEO.TEO.legs.link_lengths(1)+SETTINGS_TEO.TEO.legs.link_lengths(4); 0; 0; 0; 0];
pose_LF = [0; SETTINGS_TEO.TEO.legs.link_lengths(1)-SETTINGS_TEO.TEO.legs.link_lengths(4); 0; 0; 0; 0];

if strcmp(SETTINGS_TEO.units.pos,'m') && strcmp(SETTINGS_TEO.units.orient,'rad')
    pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(SETTINGS_TEO.h.RF_T_CoM(q0)));  % RF is the standing leg
    pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(SETTINGS_TEO.h.CoM_T_RH(q0)));
    pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(SETTINGS_TEO.h.CoM_T_LH(q0)));
elseif strcmp(SETTINGS_TEO.units.pos,'mm') && strcmp(SETTINGS_TEO.units.orient,'rad')
    temp=SETTINGS_TEO.h.RF_T_CoM(q0);
    pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));  % RF is the standing leg
    temp=SETTINGS_TEO.h.CoM_T_RH(q0);
    pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));
    temp=SETTINGS_TEO.h.CoM_T_LH(q0);
    pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));
elseif strcmp(SETTINGS_TEO.units.pos,'mm') && strcmp(SETTINGS_TEO.units.orient,'degrees')
    temp=pose_quat2rpy(SETTINGS_TEO.h.RF_T_CoM(q0));
    pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));  % RF is the standing leg
    temp=pose_quat2rpy(SETTINGS_TEO.h.CoM_T_RH(q0));
    pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));
    temp=pose_quat2rpy(SETTINGS_TEO.h.CoM_T_LH(q0));
    pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));   
elseif strcmp(SETTINGS_TEO.units.pos,'m') && strcmp(SETTINGS_TEO.units.orient,'degrees')
    temp=pose_quat2rpy(SETTINGS_TEO.h.RF_T_CoM(q0));
    pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));  % RF is the standing leg
    temp=pose_quat2rpy(SETTINGS_TEO.h.CoM_T_RH(q0));
    pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));
    temp=pose_quat2rpy(SETTINGS_TEO.h.CoM_T_LH(q0));
    pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));  
else
    disp('ERROR: No m/mm or rad/deg value')
end


trajectory = insert_trajectory(trajectory, SETTINGS_TEO.humanoid_fields, create_trajectory_structure(pose_RF, Ts, 0), 'RF');
trajectory = insert_trajectory(trajectory, SETTINGS_TEO.humanoid_fields, create_trajectory_structure(pose_LF, Ts, 0), 'LF');
trajectory = insert_trajectory(trajectory, SETTINGS_TEO.humanoid_fields, create_trajectory_structure(pose_CoM, Ts, 0), 'CoM');
trajectory = insert_trajectory(trajectory, SETTINGS_TEO.humanoid_fields, create_trajectory_structure(pose_RH, Ts, 0), 'RH');
trajectory = insert_trajectory(trajectory, SETTINGS_TEO.humanoid_fields, create_trajectory_structure(pose_LH, Ts, 0), 'LH');

SETTINGS_TEO.initial=trajectory;

% Trajectory = trajectory;
% d_Trajectory = d_trajectory;
% dd_Trajectory = dd_trajectory;
% % ------------------------------------------------------------------------- 

% if SETTINGS_TEO.body_parts.RH
%     manipulator_data.RA.W_RA = window_trajectory_RA;
% elseif SETTINGS_TEO.body_parts.LH
%     manipulator_data.LA.W_LA = window_trajectory_LA;
% elseif SETTINGS_TEO.body_parts.RF
%     manipulator_data.RL.W_RL = window_trajectory_RL;
% elseif SETTINGS_TEO.body_parts.LF
%     manipulator_data.LL.W_LL = window_trajectory_LL;
% end


guidata(hObject,handles)

trajectory_generation(SETTINGS_TEO);

close(handles.figure1)

% -------------------------- CALLBACKS ------------------------------------
% ----------- SELECTIONchangeFCN -------------------

% --- Executes when selected object is changed in position_unit.
function position_unit_SelectionChangeFcn(hObject, eventdata, handles)

global SETTINGS_TEO
option = get(eventdata.NewValue,'String');
if strcmp(option,'[mm] milimeters')
    SETTINGS_TEO.units.pos = 'mm';
else
    SETTINGS_TEO.units.pos = 'm';
end

function orientation_unit_SelectionChangeFcn(hObject, eventdata, handles)
global SETTINGS_TEO
option = get(eventdata.NewValue,'String');
if strcmp(option,'[deg] deg')
    SETTINGS_TEO.units.orient = 'degrees';
else
    SETTINGS_TEO.units.orient = 'rad';
end

function abs_diff_SelectionChangeFcn(hObject, eventdata, handles)
global SETTINGS_TEO
SETTINGS_TEO.units.def = get(eventdata.NewValue,'String');

function initial_data_panel_SelectionChangeFcn(hObject, eventdata, handles)
global SETTINGS_TEO
ini_val = get(eventdata.NewValue,'String');
switch ini_val
    case 'default position'
        SETTINGS_TEO.q0.data = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...                % Right Leg
     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...                     % Left Leg
     0.0349065850398866; 0;...                                                                                                      % Waist
     1.57079632679490; -0.167017153300893; 0; -0.734875474523928; 0; 0;                                                             % Right Arm
     1.57079632679490; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                                            % Left Arm 
 
    case 'from file'
        [FileCSV PathCSV]=uigetfile({'*.csv'}, 'Choose initial csv file (values of the motors)');
        try
            motors = csvread(FileCSV,size(load(FileCSV),1) - 1);
            %data = (motors')*conversion;
            if isempty(motors)    
                SETTINGS_TEO.q0.data = motors';
            end
            %[data(1:6);data(11:16);data(21);data(7:10);data(23);data(17:20);data(23)];
        catch
            disp('No initial csv file loaded!');
        end

        
end

% ------------ CHECKBOX -------------------

function check_filetxt_Callback(hObject, eventdata, handles)

global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.save.fdat =1;
else
    SETTINGS_TEO.save.fdat =0;
end

function check_filecsv_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.save.fcsv =1;
else
    SETTINGS_TEO.save.fcsv =0;
end

function check_LA_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.LH =1;
else
    SETTINGS_TEO.body_parts.LH  =0;
end

function check_LL_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.LF =1;
else
    SETTINGS_TEO.body_parts.LF =0;
end

function check_RA_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.RH = 1;
else
    SETTINGS_TEO.body_parts.RH =0;
    
end

function check_RL_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.RF =1;
else
    SETTINGS_TEO.body_parts.RF =0;
end

% ----------------------- Text & Edit ------------------------------------

function Ts_edit_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function Ts_edit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function kp_edit_Callback(hObject, eventdata, handles)

function kp_edit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ko_edit_Callback(hObject, eventdata, handles)

function ko_edit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over radiobutton1.
function radiobutton1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in check_CoM.
function check_CoM_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.CoM =1;
else
    SETTINGS_TEO.body_parts.CoM =0;
end


% --- Executes on button press in check_Waist.
function check_Waist_Callback(hObject, eventdata, handles)
global SETTINGS_TEO
if (get(hObject,'Value') == get(hObject,'Max'))
    SETTINGS_TEO.body_parts.Waist =1;
else
    SETTINGS_TEO.body_parts.Waist =0;
end
