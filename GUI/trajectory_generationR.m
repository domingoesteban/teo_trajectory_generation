function varargout = trajectory_generationR(varargin)
% TRAJECTORY_GENERATIONR MATLAB code for trajectory_generationR.fig
%      TRAJECTORY_GENERATIONR, by itself, creates a new TRAJECTORY_GENERATIONR or raises the existing
%      singleton*.
%
%      H = TRAJECTORY_GENERATIONR returns the handle to a new TRAJECTORY_GENERATIONR or the handle to
%      the existing singleton*.
%
%      TRAJECTORY_GENERATIONR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRAJECTORY_GENERATIONR.M with the given input arguments.
%
%      TRAJECTORY_GENERATIONR('Property','Value',...) creates a new TRAJECTORY_GENERATIONR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trajectory_generationR_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trajectory_generationR_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trajectory_generationR

% Last Modified by GUIDE v2.5 22-Nov-2013 19:21:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trajectory_generationR_OpeningFcn, ...
                   'gui_OutputFcn',  @trajectory_generationR_OutputFcn, ...
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


% --- Executes just before trajectory_generationR is made visible.
function trajectory_generationR_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trajectory_generationR (see VARARGIN)
clear trajectory d_trajectory dd_trajectory q dq ddq
global SETTINGS_TEO trajectory_points q0
global stretch_trajectory stretch_number humanoid_part total_points

% Choose default command line output for trajectory_generationR
handles.output = hObject;

if (isempty(SETTINGS_TEO) || not(isfield(SETTINGS_TEO, 'humanoid_fields')))
        settings_trajectory_generation;
        close(handles.figureTrajectoryGeneration)
        return
end

% Put the window to the center of the screen
scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);



% Joint Space Plots
graphstab = uitabpanel(...
  'Parent',handles.panel3,...
  'Style','leftbottom',...
  'Units','normalized',...
  'Position',[0,0,1,1],...
  'FrameBackgroundColor',[1,0.5,0.1],...
  'FrameBorderType','beveledin',...
  'Title',{'OPERATIONAL Space','Velocities','Accelerations','JOINT Space','Velocities','Accelerations'},...
  'PanelHeights',[8,30,10,10],...
  'HorizontalAlignment','left',...
  'FontWeight','bold',...
  'TitleBackgroundColor',[1,0.5,0],...
  'TitleForegroundColor',[0,0,0],...
  'PanelBackgroundColor',[0.5,0.5,0.5],...
  'PanelBorderType','beveledout');

handles.hpanel1 = getappdata(graphstab,'panels');

set(handles.main_menu,'Enable','on');


%%%%%%%%%%%%%%%%%%%%%%%%%%handles.newmenu = uimenu(handles.figureTrajectoryGeneration,'Label','Hola','Enable','off');
% ***********************
%%%%%%%%%%[robot current manipulator] = robot_function('TEO');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DESACTIVADOOOOOOOOOOOOOOOOO
%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

set([handles.data_panel handles.operational_panel handles.save_button handles.ik_button],'Visible','off');
set([handles.diff_pos_button handles.abs_pos_button handles.diff_orient_button handles.abs_orient_button handles.diff_time_button handles.abs_time_button],'Value',0);
set(handles.Ts_val,'String',SETTINGS_TEO.parameters.Ts);
% setappdata(hObject,'Robot',robot);   %%%%%%%%%%%%%%%%%%%%%%%%%
% setappdata(hObject,'Current',current); %%%%%%%%%%%%%%%%%%%%%%%%%%
%setappdata(hObject,'Manipulator',manipulator);
    
% ***********************

if SETTINGS_TEO.body_parts.CoM==0
    set(handles.CoM_button,'Enable','off');
end
if SETTINGS_TEO.body_parts.RF==0
    set(handles.RF_button,'Enable','off');
end
if SETTINGS_TEO.body_parts.LF==0
    set(handles.LF_button,'Enable','off');
end
if SETTINGS_TEO.body_parts.RH==0
    set(handles.RH_button,'Enable','off');
end
if SETTINGS_TEO.body_parts.LH==0
    set(handles.LH_button,'Enable','off');
end


% Set values
set ([handles.diff_pos_button, handles.diff_orient_button, handles.diff_time_button handles.legs_popup],'Value',1.0)
set (handles.num_points_text,'String',num2str(1))
stretch_trajectory={};
stretch_number=0;
stretch_trajectory{1} = get(handles.stretch_trajectory_popup,'String');
stretch_number = get(handles.stretch_trajectory_popup,'Value');
humanoid_part = 'CoM';
q0 = SETTINGS_TEO.q0.data;
total_points = 1;
% Create initial values
trajectory_points=[];
for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
    trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name) = zeros(SETTINGS_TEO.humanoid_fields(jj).size, 1);
    trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name) = zeros(SETTINGS_TEO.humanoid_fields(jj).size, 1);
    trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name) = 0;
    trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name) = 0;
    trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name) = SETTINGS_TEO.parameters.Ts;
    trajectory_points.pos_diff.(SETTINGS_TEO.humanoid_fields(jj).name){1} = 'Diff';
    trajectory_points.orient_diff.(SETTINGS_TEO.humanoid_fields(jj).name){1} = 'Diff';
    trajectory_points.time_diff.(SETTINGS_TEO.humanoid_fields(jj).name){1} = 'Diff';
    trajectory_points.interpola_pos.(SETTINGS_TEO.humanoid_fields(jj).name){1} = 'Linear';
    trajectory_points.interpola_orient.(SETTINGS_TEO.humanoid_fields(jj).name){1} = 'Linear';
    trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name) = 0;
end




% Update handles structure

   axes(handles.axesUC3M)
   [r,map]=imread('uc3m.png','png');
   image(r);colormap(map);axis off
   
guidata(hObject, handles);

% UIWAIT makes trajectory_generationR wait for user response (see UIRESUME)
% uiwait(handles.figureTrajectoryGeneration);


% --- Outputs from this function are returned to the command line.
function varargout = trajectory_generationR_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles;


% --- Executes on selection change in legs_popup.
function legs_popup_Callback(hObject, eventdata, handles)
update_support_foot(handles);
messages_master(handles)
guidata(hObject,handles)


% Hints: contents = cellstr(get(hObject,'String')) returns legs_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from legs_popup


% --- Executes during object creation, after setting all properties.
function legs_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to legs_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t0_text_Callback(hObject, eventdata, handles)
% hObject    handle to t0_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t0_text as text
%        str2double(get(hObject,'String')) returns contents of t0_text as a double


% --- Executes during object creation, after setting all properties.
function t0_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t0_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_text_Callback(hObject, eventdata, handles)
% hObject    handle to T_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T_text as text
%        str2double(get(hObject,'String')) returns contents of T_text as a double


% --- Executes during object creation, after setting all properties.
function T_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t0_val_Callback(hObject, eventdata, handles)
% hObject    handle to t0_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t0_val as text
%        str2double(get(hObject,'String')) returns contents of t0_val as a double


% --- Executes during object creation, after setting all properties.
function t0_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t0_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_val_Callback(hObject, eventdata, handles)
% hObject    handle to T_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T_val as text
%        str2double(get(hObject,'String')) returns contents of T_val as a double


% --- Executes during object creation, after setting all properties.
function T_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ts_val_Callback(hObject, eventdata, handles)
% hObject    handle to Ts_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ts_val as text
%        str2double(get(hObject,'String')) returns contents of Ts_val as a double


% --- Executes during object creation, after setting all properties.
function Ts_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ts_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in interpola_pos_popup.
function interpola_pos_popup_Callback(hObject, eventdata, handles)
% hObject    handle to interpola_pos_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global trajectory_points humanoid_part stretch_number
contents = cellstr(get(hObject,'String'));
trajectory_points.interpola_pos.(humanoid_part){stretch_number}=contents{get(hObject,'Value')};
if contents{get(hObject,'Value')}==2
    set(handles.label_points,'String','No Circular interpolation implemented jet!');
    set(handles.label_points,'BackGroundColor',[1 0.2 0.2]);
else
    set(handles.label_points,'String','');
    set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
end


% global p_via dp_ini dp_end ddp_ini ddp_end
% 
% p_via = zeros(3,1);dp_ini = zeros(3,1);dp_end = zeros(3,1);ddp_ini = zeros(3,1);ddp_end = zeros(3,1);
% p_ini = [str2double(get(handles.xi_val,'String'));str2double(get(handles.yi_val,'String'));str2double(get(handles.zi_val,'String'))];
% p_end = [str2double(get(handles.xf_val,'String'));str2double(get(handles.yf_val,'String'));str2double(get(handles.zf_val,'String'))];
% contents = cellstr(get(hObject,'String'));
% switch contents{get(hObject,'Value')}
%     case 'Linear'
%         
%     case 'Circular'
%        circular_points(p_ini,p_end)
%        uiwait
%     case 'Spline'
%        spline_points({'v0 x' 'v0 y' 'v0 z';'Vf x' 'Vf y' 'Vf z'},{'a0 x' 'a0 y' 'a0 z';'Af x' 'Af y' 'Af z'});
%        uiwait 
%     case 'Polynomial'
%        polynomial_points({'v0 x' 'v0 y' 'v0 z';'Vf x' 'Vf y' 'Vf z'});
%        uiwait
% end
% diffabs = handles.diffabs;
% guidata(hObject,handles)
% vel_acel_func(hObject,handles,1,diffabs,contents{get(hObject,'Value')}

% Hints: contents = cellstr(get(hObject,'String')) returns interpola_pos_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from interpola_pos_popup



% --- Executes during object creation, after setting all properties.
function interpola_pos_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpola_pos_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xf_val_Callback(hObject, eventdata, handles)
% hObject    handle to xf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xf_val as text
%        str2double(get(hObject,'String')) returns contents of xf_val as a double


% --- Executes during object creation, after setting all properties.
function xf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yf_val_Callback(hObject, eventdata, handles)
% hObject    handle to yf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yf_val as text
%        str2double(get(hObject,'String')) returns contents of yf_val as a double


% --- Executes during object creation, after setting all properties.
function yf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zf_val_Callback(hObject, eventdata, handles)
% hObject    handle to zf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zf_val as text
%        str2double(get(hObject,'String')) returns contents of zf_val as a double


% --- Executes during object creation, after setting all properties.
function zf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xi_val_Callback(hObject, eventdata, handles)
% hObject    handle to xi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xi_val as text
%        str2double(get(hObject,'String')) returns contents of xi_val as a double


% --- Executes during object creation, after setting all properties.
function xi_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yi_val_Callback(hObject, eventdata, handles)
% hObject    handle to yi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yi_val as text
%        str2double(get(hObject,'String')) returns contents of yi_val as a double


% --- Executes during object creation, after setting all properties.
function yi_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zi_val_Callback(hObject, eventdata, handles)
% hObject    handle to zi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zi_val as text
%        str2double(get(hObject,'String')) returns contents of zi_val as a double


% --- Executes during object creation, after setting all properties.
function zi_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in interpola_orient_popup.
function interpola_orient_popup_Callback(hObject, eventdata, handles)
% hObject    handle to interpola_orient_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global trajectory_points humanoid_part stretch_number
contents = cellstr(get(hObject,'String'));
trajectory_points.interpola_orient.(humanoid_part){stretch_number}=contents{get(hObject,'Value')};

% global dp_ini dp_end ddp_ini ddp_end
% 
% dp_ini = zeros(3,1);dp_end = zeros(3,1);ddp_ini = zeros(3,1);ddp_end = zeros(3,1);
% 
% contents = cellstr(get(hObject,'String'));
% switch contents{get(hObject,'Value')}
%     case 'Linear'
%         
%     case 'Spline'
%        spline_points({'v0 roll' 'v0 pitch' 'v0 yaw';'Vf roll' 'Vf pitch' 'Vf yaw'},{'a0 row' 'a0 pitch' 'a0 yaw';'Af roll' 'Af pitch' 'Af yaw'});
%        uiwait 
%     case 'Polynomial'
%        polynomial_points({'v0 roll' 'v0 pitch' 'v0 yaw';'Vf roll' 'Vf pitch' 'Vf yaw'});
%        uiwait
% end
% diffabs = handles.diffabs;
% guidata(hObject,handles)
% vel_acel_func(hObject,handles,2,diffabs,contents{get(hObject,'Value')})
% Hints: contents = cellstr(get(hObject,'String')) returns interpola_orient_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from interpola_orient_popup


% --- Executes during object creation, after setting all properties.
function interpola_orient_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpola_orient_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yawf_val_Callback(hObject, eventdata, handles)
% hObject    handle to yawf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yawf_val as text
%        str2double(get(hObject,'String')) returns contents of yawf_val as a double


% --- Executes during object creation, after setting all properties.
function yawf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yawf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitchf_val_Callback(hObject, eventdata, handles)
% hObject    handle to pitchf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitchf_val as text
%        str2double(get(hObject,'String')) returns contents of pitchf_val as a double


% --- Executes during object creation, after setting all properties.
function pitchf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitchf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rollf_val_Callback(hObject, eventdata, handles)
% hObject    handle to rollf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rollf_val as text
%        str2double(get(hObject,'String')) returns contents of rollf_val as a double


% --- Executes during object creation, after setting all properties.
function rollf_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rollf_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rolli_val_Callback(hObject, eventdata, handles)
% hObject    handle to rolli_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rolli_val as text
%        str2double(get(hObject,'String')) returns contents of rolli_val as a double


% --- Executes during object creation, after setting all properties.
function rolli_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rolli_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitchi_val_Callback(hObject, eventdata, handles)
% hObject    handle to pitchi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitchi_val as text
%        str2double(get(hObject,'String')) returns contents of pitchi_val as a double


% --- Executes during object creation, after setting all properties.
function pitchi_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitchi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yawi_val_Callback(hObject, eventdata, handles)
% hObject    handle to yawi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yawi_val as text
%        str2double(get(hObject,'String')) returns contents of yawi_val as a double


% --- Executes during object creation, after setting all properties.
function yawi_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yawi_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in gen_button.
function gen_button_Callback(hObject, eventdata, handles)
clear trajectory d_trajectory dd_trajectory
global trajectory d_trajectory dd_trajectory
global trajectory_points SETTINGS_TEO total_points humanoid_part
% hObject    handle to gen_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global p_via
% current = getappdata(handles.figureTrajectoryGeneration,'Current');
% if strcmpi(current.position.diffabs,'Diff')
%     current.position.pf = current.position.p0 + [str2double(get(handles.xf_val,'String'));str2double(get(handles.yf_val,'String'));str2double(get(handles.zf_val,'String'))];
% else
%     current.position.pf = [str2double(get(handles.xf_val,'String'));str2double(get(handles.yf_val,'String'));str2double(get(handles.zf_val,'String'))];
% end
% current.delta_pos = current.position.pf - current.position.p0;
% if strcmpi(current.orientation.diffabs,'Diff')
%     current.orientation.af = current.orientation.a0 + [str2double(get(handles.rollf_val,'String'));str2double(get(handles.pitchf_val,'String'));str2double(get(handles.yawf_val,'String'))];
% else
%     current.orientation.pf = [str2double(get(handles.rollf_val,'String'));str2double(get(handles.pitchf_val,'String'));str2double(get(handles.yawf_val,'String'))];
% end
% current.delta_orient = current.orientation.af - current.orientation.a0;
% current.time.t0 = str2double(get(handles.t0_val,'String'));
% current.time.Ts = str2double(get(handles.Ts_val,'String'));
% if strcmpi(current.time.diffabs,'Diff')
%     current.time.T = current.time.t0 + str2double(get(handles.T_val,'String'));
% else
%     current.time.T = str2double(get(handles.T_val,'String'));
% end
% current.delta_orient = current.orientation.af - current.orientation.a0;
% 
% current.manipulator{1} = strcat('trajectory',' ',current.endeffector);
% 
% [trajectory,d_trajectory,dd_trajectory] = gen_trajectory(current,p_via);
% current.manipulator{2} = current.time;
% current.manipulator{3} = trajectory;
% current.manipulator{4} = d_trajectory;
% current.manipulator{5} = dd_trajectory;
% setappdata(handles.figureTrajectoryGeneration,'Current',current);
% 


update_points(humanoid_part,handles);
update_support_foot(handles);
total_points = str2double(get(handles.num_points_text,'String'));
[trajectory d_trajectory dd_trajectory]=generate_trajectory(trajectory, d_trajectory, dd_trajectory, trajectory_points, total_points, SETTINGS_TEO);
set([handles.save_button handles.joint_panel handles.traj_name_panel handles.ik_button handles.visualize_axis_movement_pushbutton],'Visible','on');
set(handles.save_ik_button,'Visible','off')
gen_traj_graphs_function(hObject,handles,trajectory,d_trajectory,dd_trajectory)

guidata(hObject,handles)

% --- Executes on button press in save_button.
function save_button_Callback(hObject, eventdata, handles)
global trajectory d_trajectory dd_trajectory humanoid_part
global save_trajectory save_d_trajectory save_dd_trajectory traj_name_string
traj_name_string = get(handles.traj_name_text,'String');
save_trajectory = trajectory.(humanoid_part);
save_d_trajectory = d_trajectory.(humanoid_part);
save_dd_trajectory = dd_trajectory.(humanoid_part);
save_trajectory_window;
guidata(hObject,handles)


% --- Executes on button press in ik_button.
function ik_button_Callback(hObject, eventdata, handles)
% hObject    handle to ik_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% current = getappdata(handles.figureTrajectoryGeneration,'Current');
% robot = getappdata(handles.figureTrajectoryGeneration,'Robot');
% [q, dq, ddq] = ik_manipulator(current,robot.h);
% 
% current.manipulator(6:8) = {q,dq,ddq};
% 
% ik_graphs_function(hObject,handles,q,dq,ddq,current,robot)
% 
% 
% setappdata(handles.figureTrajectoryGeneration,'Current',current);
clear q dq ddq
global q0 trajectory d_trajectory dd_trajectory SETTINGS_TEO humanoid_part
global q dq ddq
set(handles.save_button,'ForegroundColor','black');
set(handles.label_plots,'String','Calculating Inverse kinematics. Please wait...');
set(handles.label_plots,'BackGroundColor',[1 1 0.2]);

[q, dq, ddq] = ik_TEO2(q0, trajectory, d_trajectory, dd_trajectory, SETTINGS_TEO.h, SETTINGS_TEO.parameters, SETTINGS_TEO, humanoid_part);
                        
set(handles.label_plots,'String','Inverse kinematics Complete!');
set(handles.label_plots,'BackGroundColor',[0.702 0.78 1]);
set(handles.save_button,'ForegroundColor','red');
ik_graphs_function(hObject,handles,q,dq,ddq);
set([handles.save_ik_button handles.visualize_humanoid_movement_pushbutton],'Visible','on');



% --- Executes when selected object is changed in humanoid_part_panel.
function humanoid_part_panel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in humanoid_part_panel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
global humanoid_part old_humanoid_part 
global trajectory d_trajectory dd_trajectory q dq ddq
new_humanoid_part = get(eventdata.NewValue,'String');
old_humanoid_part = get(eventdata.OldValue,'String');
set(handles.panel3,'Title',new_humanoid_part);
%set(handles.data_panel,'Visible','on');

%Update previous humanoid_part value
if ~(isempty(old_humanoid_part))
    switch old_humanoid_part
        case 'Right Hand'
            old_humanoid_part = 'RH';
        case 'Left Hand'
            old_humanoid_part = 'LH';
        case 'Right Foot'
            old_humanoid_part = 'RF';
        case 'Left Foot'
            old_humanoid_part = 'LF';
    end
    update_points(old_humanoid_part,handles);
    update_support_foot(handles);
end

%Update operational panel
initial_val_function (hObject,handles)
switch new_humanoid_part
    case 'Right Hand'
            humanoid_part = 'RH';
        
    case 'Left Hand'
        
        humanoid_part = 'LH'; 
        
    case 'Right Foot'
       
        humanoid_part = 'RF';
        
    case 'Left Foot'
        
        humanoid_part = 'LF';
        
    case 'CoM'
        
        humanoid_part = 'CoM';
end

add_default_data()
messages_master(handles)
show_selected_values(handles)

% Show graphs only if trajectory exists (there are more than one point)
if size(trajectory.(humanoid_part),2)>1
    gen_traj_graphs_function(hObject,handles,trajectory,d_trajectory,dd_trajectory);
end
if (exist('q') && not(isempty(q)))
    ik_graphs_function(hObject,handles,q,dq,ddq);
end
guidata(hObject, handles);


% --- Executes when selected object is changed in data_panel.
function data_panel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in data_panel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
% [FileCSV PathCSV]=uigetfile({'*.csv'}, 'Choose initial csv file');
% 
% motors = csvread(FileCSV,size(load(FileCSV),1) - 1);
% 
% 
% initial_data = (motors')*conversion
global joint_val
new_val = get(eventdata.NewValue,'String');
old_val = get(eventdata.OldValue,'String');
new_part = handles.new_humanoid_part;
current = getappdata(handles.figureTrajectoryGeneration,'Current');
robot = getappdata(handles.figureTrajectoryGeneration,'Robot');
current.datafrom = new_val;
current.endeffector = new_part;
switch new_val
    case 'File'
        
    case 'Default'
        current.q0 = robot.default.q0;
        current.time.T0 = robot.default.t0;
        current.time.Ts = robot.default.Ts;
        current.time.T = robot.default.T;
    case 'Joint Insert'
        joint_insert(new_part,robot)
        uiwait
        
        current.q0 = joint_val;
        current.time.T0 = robot.default.t0;
        current.time.Ts = robot.default.Ts;
        current.time.T = robot.default.T;
        
end
switch new_part
    case 'RH'
       PO = pose_quat2rpy(robot.h.CoM_T_RH(current.q0)); 
    case 'LH'
       PO = pose_quat2rpy(robot.h.CoM_T_LH(current.q0)); 
    case 'RLF'
       PO = zeros(6,1);
       %PO = pose_quat2rpy(robot.h.CoM_T_RF(current.q0)); 
    case 'LLF'
       PO = zeros(6,1);
       %PO = pose_quat2rpy(robot.h.CoM_T_LF(current.q0)); 
    case 'RLS'
       PO = pose_quat2rpy(robot.h.RF_T_CoM(current.q0)); 
    case 'LLS'
       PO = pose_quat2rpy(robot.h.LF_T_CoM(current.q0));
end
current.position.p0 = PO(1:3,1);
current.orientation.a0 = PO(4:6,1);
set([handles.operational_panel handles.save_button handles.ik_button],'Visible','on');
% set(handles.xi_val,'String',num2str(current.position.p0(1)));
% set(handles.yi_val,'String',num2str(current.position.p0(2)));
% set(handles.zi_val,'String',num2str(current.position.p0(3)));
% set(handles.rolli_val,'String',num2str(current.orientation.a0(1)));
% set(handles.pitchi_val,'String',num2str(current.orientation.a0(2)));
% set(handles.yawi_val,'String',num2str(current.orientation.a0(3)));
% set(handles.t0_val,'String',num2str(current.time.t0));
% set(handles.Ts_val,'String',num2str(current.time.Ts));
% 
% setappdata(handles.figureTrajectoryGeneration,'Current',current);
% setappdata(handles.figureTrajectoryGeneration,'Robot',robot);
guidata(hObject,handles)


% --- Executes when selected object is changed in pos_diffabs_panel.
function pos_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in pos_diffabs_panel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
global humanoid_part trajectory_points stretch_number
set([handles.xf_val handles.yf_val handles.zf_val],'String','0');

trajectory_points.pos_diff.(humanoid_part){stretch_number}=get(eventdata.NewValue,'String');

switch trajectory_points.pos_diff.(humanoid_part){stretch_number}
    case 'Diff'
        set(handles.xf_text ,'String','delta x');set(handles.yf_text,'String','delta y');set(handles.zf_text,'String','delta z');
    case 'Abs'
        set(handles.xf_text ,'String','x f');set(handles.yf_text,'String','y f');set(handles.zf_text,'String','z f');
end
%handles.diffabs = get(eventdata.NewValue,'String');


guidata(hObject,handles)


% --- Executes when selected object is changed in orient_diffabs_panel.
function orient_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in orient_diffabs_panel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
global humanoid_part trajectory_points stretch_number
set([handles.rollf_val handles.pitchf_val handles.yawf_val],'String','0');

% trajectory_points.time_diff trajectory_points.interpola_pos trajectory_points.interpola_orient
% trajectory_points.pos_diff.(new_humanoid_part)=get(eventdata.NewValue,'String');

trajectory_points.orient_diff.(humanoid_part){stretch_number}=get(eventdata.NewValue,'String');

switch trajectory_points.orient_diff.(humanoid_part){stretch_number}
    case 'Diff'
        set(handles.rollf_text ,'String','delta roll');set(handles.pitchf_text,'String','delta pitch');set(handles.yawf_text,'String','delta yaw');
    case 'Abs'
        set(handles.rollf_text ,'String','roll f');set(handles.pitchf_text,'String','pitch f');set(handles.yawf_text,'String','yaw f');
end
handles.diffabs = get(eventdata.NewValue,'String');
guidata(hObject,handles)


% --- Executes when selected object is changed in time_diffabs_panel.
function time_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in time_diffabs_panel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
global humanoid_part trajectory_points stretch_number
set(handles.T_val,'String','0');

trajectory_points.time_diff.(humanoid_part){stretch_number}=get(eventdata.NewValue,'String');

switch trajectory_points.time_diff.(humanoid_part){stretch_number}
    case 'Diff'
        set(handles.T_text ,'String','Delta Time');
    case 'Abs'
        set(handles.T_text ,'String','Final Time');
end
handles.diffabs = get(eventdata.NewValue,'String');
guidata(hObject,handles)

function vel_acel_func(hOject,handles,caso,diffabs,interpola)
%
%
global dp_ini dp_end ddp_ini ddp_end
current = getappdata(handles.figureTrajectoryGeneration,'Current');
switch caso
    case 1
        current.position.diffabs = diffabs;

        current.position.dp0 = dp_ini;
        current.position.ddp0 = ddp_ini;

        current.position.dpf = dp_end;
        current.position.ddpf = ddp_end;
        current.interpola.pos = interpola;
    
    case 2
        current.orientation.diffabs = diffabs;

        current.orientation.da0 = dp_ini;
        current.orientation.dda0 = ddp_ini;

        current.orientation.daf = dp_end;
        current.orientation.ddaf = ddp_end;
        current.interpola.orient = interpola;
end
setappdata(handles.figureTrajectoryGeneration,'Current',current);



function traj_name_text_Callback(hObject, eventdata, handles)
% hObject    handle to traj_name_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of traj_name_text as text
%        str2double(get(hObject,'String')) returns contents of traj_name_text as a double


% --- Executes during object creation, after setting all properties.
function traj_name_text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function initial_val_function (hObject,handles)
%
cla(handles.operational_axes1); cla(handles.operational_axes2);cla(handles.operational_axes3);cla(handles.operational_axes4);
cla(handles.operational_axes5);cla( handles.operational_axes6);
% set([handles.xi_val handles.yi_val handles.zi_val handles.xf_val handles.yf_val handles.zf_val ...
%     handles.rolli_val handles.pitchi_val handles.yawi_val handles.rollf_val handles.pitchf_val ...
%     handles.yawf_val handles.t0_val handles.T_val],'String',0);
set(handles.traj_name_text,'String',' ');
set(handles.save_button,'ForegroundColor','black');
set([handles.default_button handles.file_button handles.joint_button...
    handles.diff_pos_button handles.abs_pos_button handles.diff_orient_button...
    handles.abs_orient_button handles.diff_time_button handles.abs_time_button],'Value',0);
guidata(hObject,handles)




function gen_traj_graphs_function(hObject,handles,trajectory,d_trajectory,dd_trajectory)
%
global humanoid_part SETTINGS_TEO
time = trajectory.time;
set(handles.graph_panel1,'Parent',handles.hpanel1(1));
set(handles.graph_panel2,'Parent',handles.hpanel1(2));
set(handles.graph_panel3,'Parent',handles.hpanel1(3));
titulos = {'X' 'Y' 'Z' 'roll' 'pitch' 'yaw'};
titulos_y = {'[m]' '[m]' '[m]' '[rad]' '[rad]' '[rad]'...
    '[m/s]' '[m/s]' '[m/s]' '[rad/s]' '[rad/s]' '[rad/s]'...
    '[m/s^2]' '[m/s^2]' '[m/s^2]' '[rad/s^2]' '[rad/s^2]' '[rad/s^2]'};

for jj = 1:6
    axes(handles.(strcat('operational_axes',num2str(jj))))
    cla(handles.(strcat('operational_axes',num2str(jj))),'reset')
    plot(handles.(strcat('operational_axes',num2str(jj))),trajectory.(humanoid_part)(jj,:));hold on;
    
    title(handles.(strcat('operational_axes',num2str(jj))),titulos(jj))
    xlabel(handles.(strcat('operational_axes',num2str(jj))),'t [s]')
    ylabel(handles.(strcat('operational_axes',num2str(jj))),titulos_y(jj))
    hold off;
    
    axes(handles.(strcat('operational_axes',num2str(jj+6))))
    cla(handles.(strcat('operational_axes',num2str(jj+6))),'reset')
    plot(handles.(strcat('operational_axes',num2str(jj+6))),d_trajectory.(humanoid_part)(jj,:));hold on;
    title(handles.(strcat('operational_axes',num2str(jj+6))),strcat('velocity ',' ',titulos(jj)))
    xlabel(handles.(strcat('operational_axes',num2str(jj+6))),'t [s]')
    ylabel(handles.(strcat('operational_axes',num2str(jj+6))),titulos_y(jj+6))
    hold off;
    
    axes(handles.(strcat('operational_axes',num2str(jj+12))))
    cla(handles.(strcat('operational_axes',num2str(jj+12))),'reset')
    plot(handles.(strcat('operational_axes',num2str(jj+12))),dd_trajectory.(humanoid_part)(jj,:));hold on;
    title(handles.(strcat('operational_axes',num2str(jj+12))),strcat('acceleration ',' ',titulos(jj)))
    xlabel(handles.(strcat('operational_axes',num2str(jj+12))),'t [s]')
    ylabel(handles.(strcat('operational_axes',num2str(jj+12))),titulos_y(jj+12))
    hold off;
end
guidata(hObject,handles)

function ik_graphs_function(hObject,handles,q,dq,ddq)
%
global humanoid_part SETTINGS_TEO

set(handles.label_plots,'String','Plotting Inverse Kinematics. Please wait...');
set(handles.label_plots,'BackGroundColor',[1 1 0.2]);

set(handles.graph_panel4,'Parent',handles.hpanel1(4));
set(handles.graph_panel5,'Parent',handles.hpanel1(5));
set(handles.graph_panel6,'Parent',handles.hpanel1(6));
switch humanoid_part
    case 'RH'
        qq = q(15:20,:);
        dqq = dq(15:20,:);
        ddqq = ddq(15:20,:);
        j_limit1 = 'arms';j_limit2 = 'right';
    case 'LH'
        qq = q(21:26,:);
        dqq = dq(21:26,:);
        ddqq = ddq(21:26,:);
        j_limit1 = 'arms';j_limit2 = 'left';
    case 'RF'
        qq = q(1:6,:);
        dqq = dq(1:6,:);
        ddqq = ddq(1:6,:);
        j_limit1 = 'legs';j_limit2 = 'right';
    case 'LF'
        qq = q(7:12,:);
        dqq = dq(7:12,:);
        ddqq = ddq(7:12,:);
        j_limit1 = 'legs';j_limit2 = 'left';
    case 'CoM'
        qq = q(13:14,:);
        dqq = dq(13:14,:);
        ddqq = ddq(13:14,:);
        j_limit1 = 'waist';
end
[m,n]=size(qq);
for jj=1:m
    if strcmp(humanoid_part,'CoM')
        limits =  SETTINGS_TEO.TEO.(j_limit1).joint(jj).angle_limits;
        titlesik =  SETTINGS_TEO.TEO.(j_limit1).joint(jj).name; 
    else
        limits =  SETTINGS_TEO.TEO.(j_limit1).(j_limit2).joint(jj).angle_limits;
        titlesik =  SETTINGS_TEO.TEO.(j_limit1).(j_limit2).joint(jj).name; 
    end
    axes(handles.(strcat('joint_axes',num2str(jj))))
    cla(handles.(strcat('joint_axes',num2str(jj))),'reset')
    plot(limits(1,1)*ones(n,1),'r');hold on;
    plot(limits(1,2)*ones(n,1),'r');
    plot(qq(jj,:));hold on;
    title(titlesik)
    xlabel('t [s]')
    ylabel('[rad]')
    plot(limits(1,1)*ones(n,1),'r');
    plot(limits(1,2)*ones(n,1),'r');
    hold off
    
    %axes(handles.(strcat('joint_axes',num2str(jj+6))))
    plot(handles.(strcat('joint_axes',num2str(jj+6))),dqq(jj,:));hold on;
    title(handles.(strcat('joint_axes',num2str(jj+6))),titlesik)
    xlabel(handles.(strcat('joint_axes',num2str(jj+6))),'t [s]')
    ylabel(handles.(strcat('joint_axes',num2str(jj+6))),'[rad/s]')
    
    %axes(handles.(strcat('joint_axes',num2str(jj+12))))
    plot(handles.(strcat('joint_axes',num2str(jj+12))),ddqq(jj,:));hold on;
    title(handles.(strcat('joint_axes',num2str(jj+12))),titlesik)
    xlabel(handles.(strcat('joint_axes',num2str(jj+12))),'t [s]')
    ylabel(handles.(strcat('joint_axes',num2str(jj+12))),'[rad/s^2]')
end
if strcmp(humanoid_part,'CoM')
%clear other plots if humanoid_part is CoM
    for kk=3:6,
        axes(handles.(strcat('joint_axes',num2str(kk))))
        cla(handles.(strcat('joint_axes',num2str(kk))),'reset')
        axes(handles.(strcat('joint_axes',num2str(kk+6))))
        cla(handles.(strcat('joint_axes',num2str(kk+6))),'reset')
        axes(handles.(strcat('joint_axes',num2str(kk+12))))
        cla(handles.(strcat('joint_axes',num2str(kk+12))),'reset')
    end;
end

set(handles.label_plots,'String','Plotting Inverse Kinematics Complete!');
set(handles.label_plots,'BackGroundColor',[0.702 0.78 1]);
guidata(hObject,handles)


% --------------------------------------------------------------------
function main_menu_Callback(hObject, eventdata, handles)
% hObject    handle to main_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd ..
main_TEO
close trajectory_generation


% --------------------------------------------------------------------
function Back_Callback(hObject, eventdata, handles)
% hObject    handle to Back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function trajectory_settings_Callback(hObject, eventdata, handles)
% hObject    handle to trajectory_settings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
settings_trajectory_generation
close trajectory_generation


% --------------------------------------------------------------------
function show_selected_values(handles)
    global humanoid_part stretch_trajectory stretch_number SETTINGS_TEO
    global trajectory trajectory_points
    set(handles.operational_panel,'Visible','on');
    set(handles.operational_panel,'Title',['Operational Space - ' '"Stretch ' stretch_trajectory{stretch_number} '" - ' humanoid_part ' Pose']);
    add_default_data();
    
% Initial Point
if stretch_number==1
    set(handles.xi_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(1)));
    set(handles.yi_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(2)));
    set(handles.zi_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(3)));
    set(handles.rolli_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(4)));
    set(handles.pitchi_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(5)));
    set(handles.yawi_val,'String', num2str(SETTINGS_TEO.initial.(humanoid_part)(6)));
else
    if strcmp(trajectory_points.orient_diff.(humanoid_part){stretch_number-1},'Diff')
        set(handles.xi_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(1,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(1,stretch_number-1)));
        set(handles.yi_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(2,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(2,stretch_number-1)));
        set(handles.zi_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(3,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(3,stretch_number-1)));
        set(handles.rolli_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(4,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(4,stretch_number-1)));
        set(handles.pitchi_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(5,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(5,stretch_number-1)));
        set(handles.yawi_val,'String', num2str(trajectory_points.initial_point.(humanoid_part)(6,stretch_number-1)+trajectory_points.final_point.(humanoid_part)(6,stretch_number-1)));
    elseif strcmp(trajectory_points.orient_diff.(humanoid_part){stretch_number-1},'Abs')
        set(handles.xi_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(1,stretch_number-1)));
        set(handles.yi_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(2,stretch_number-1)));
        set(handles.zi_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(3,stretch_number-1)));
        set(handles.rolli_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(4,stretch_number-1)));
        set(handles.pitchi_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(5,stretch_number-1)));
        set(handles.yawi_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(6,stretch_number-1)));
    else
       disp('ERROR: No diff/abs orientation option')
    end
end
% Final Point
    set(handles.xf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(1,stretch_number)));
    set(handles.yf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(2,stretch_number)));
    set(handles.zf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(3,stretch_number)));
    set(handles.rollf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(4,stretch_number)));
    set(handles.pitchf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(5,stretch_number)));
    set(handles.yawf_val,'String', num2str(trajectory_points.final_point.(humanoid_part)(6,stretch_number))); 
            
%Diff or Abs
    if strcmp(trajectory_points.pos_diff.(humanoid_part)(stretch_number),'Diff')
        set (handles.diff_pos_button,'Value',1.0);
    else
        set (handles.abs_pos_button,'Value',1.0);
    end
    if strcmp(trajectory_points.orient_diff.(humanoid_part)(stretch_number),'Diff')
        set (handles.diff_orient_button,'Value',1.0);
    else
        set (handles.abs_orient_button,'Value',1.0);
    end
    if strcmp(trajectory_points.time_diff.(humanoid_part)(stretch_number),'Diff')
        set (handles.diff_time_button,'Value',1.0);
    else
        set (handles.abs_time_button,'Value',1.0);
    end
    
% Times
    set (handles.T_val,'String',num2str(trajectory_points.T_val.(humanoid_part)(stretch_number)))
    if stretch_number==1
        set (handles.t0_val,'String',num2str(trajectory_points.t0_val.(humanoid_part)(stretch_number)));
        set (handles.Ts_val,'String',num2str(trajectory_points.Ts_val.(humanoid_part)(stretch_number)));
    else
        set (handles.Ts_val,'String',num2str(trajectory_points.Ts_val.(humanoid_part)(stretch_number-1))) 
        if strcmp(trajectory_points.orient_diff.(humanoid_part){stretch_number-1},'Diff')
            set (handles.t0_val,'String',num2str(trajectory_points.t0_val.(humanoid_part)(stretch_number-1)+trajectory_points.T_val.(humanoid_part)(stretch_number-1)));
        elseif strcmp(trajectory_points.orient_diff.(humanoid_part){stretch_number-1},'Abs')
            set (handles.t0_val,'String',num2str(trajectory_points.T_val.(humanoid_part)(stretch_number-1)));
        else
            disp('ERROR: No diff/abs time option')
        end  
    end
           

% Interpolations
switch trajectory_points.interpola_pos.(humanoid_part){stretch_number}
    case 'Linear'
        set(handles.interpola_pos_popup,'Value',1);
    case 'Circular'
        set(handles.interpola_pos_popup,'Value',2);
    case 'Spline'
        set(handles.interpola_pos_popup,'Value',3);
    case 'Polynomial3'
        set(handles.interpola_pos_popup,'Value',4);
    case 'Cubic Spline'
        set(handles.interpola_pos_popup,'Value',5);
end

switch trajectory_points.interpola_orient.(humanoid_part){stretch_number}
    case 'Linear'
        set(handles.interpola_orient_popup,'Value',1);
    case 'Spline'
        set(handles.interpola_orient_popup,'Value',2);
    case 'Polynomial3'
        set(handles.interpola_orient_popup,'Value',3);
    case 'Cubic Spline'
        set(handles.interpola_orient_popup,'Value',4);
end
% Support foot
switch trajectory_points.support_foot.(humanoid_part)(stretch_number)
    case 0
        set(handles.legs_popup,'Value',1);
    case -1
        set(handles.legs_popup,'Value',2);
    case 1
        set(handles.legs_popup,'Value',3);
end
   
   
    
    
function update_points(humanoid_part,handles)
global stretch_number
global trajectory_points SETTINGS_TEO

    trajectory_points.initial_point.(humanoid_part)(1,stretch_number) = str2num(get(handles.xi_val,'String'));
    trajectory_points.initial_point.(humanoid_part)(2,stretch_number) = str2num(get(handles.yi_val,'String'));
    trajectory_points.initial_point.(humanoid_part)(3,stretch_number) = str2num(get(handles.zi_val,'String'));
    trajectory_points.initial_point.(humanoid_part)(4,stretch_number) = str2num(get(handles.rolli_val,'String'));
    trajectory_points.initial_point.(humanoid_part)(5,stretch_number) = str2num(get(handles.pitchi_val,'String'));
    trajectory_points.initial_point.(humanoid_part)(6,stretch_number) = str2num(get(handles.yawi_val,'String'));
    trajectory_points.final_point.(humanoid_part)(1,stretch_number) = str2num(get(handles.xf_val,'String'));
    trajectory_points.final_point.(humanoid_part)(2,stretch_number) = str2num(get(handles.yf_val,'String'));
    trajectory_points.final_point.(humanoid_part)(3,stretch_number) = str2num(get(handles.zf_val,'String'));
    trajectory_points.final_point.(humanoid_part)(4,stretch_number) = str2num(get(handles.rollf_val,'String'));
    trajectory_points.final_point.(humanoid_part)(5,stretch_number) = str2num(get(handles.pitchf_val,'String'));
    trajectory_points.final_point.(humanoid_part)(6,stretch_number) = str2num(get(handles.yawf_val,'String'));

    trajectory_points.t0_val.(humanoid_part)(stretch_number) = str2num(get(handles.t0_val,'String'));
    trajectory_points.Ts_val.(humanoid_part)(stretch_number) = str2num(get(handles.Ts_val,'String'));
    
    %All humanoid_parts will have the same T value
    for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
    trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name)(stretch_number) = str2num(get(handles.T_val,'String'));
    end
    



function num_points_text_Callback(hObject, eventdata, handles)
global total_points stretch_trajectory
total_points = str2double(get(hObject,'String'));

% Check the correct number of points
if ((total_points<1)||(round(total_points) ~= total_points))
    warndlg('Error!: Number of points has to be an integer greater than 1','Points Error');
    total_points=floor(total_points);
    if total_points<1
        set(hObject,'String',num2str(1));
    else
        set(hObject,'String',num2str(total_points));
    end
end
% Set the values of the stretch_trajectory_popup values
stretch_trajectory={};
for jj=1:total_points
    stretch_trajectory{jj} = [num2str(jj-1) ' - ' num2str(jj)];
end
set(handles.stretch_trajectory_popup,'String',stretch_trajectory)
set(handles.stretch_trajectory_popup,'Value',1);
set(handles.CoM_button,'Value',1.0)
add_default_data()
stretch_trajectory_popup_Callback(handles.stretch_trajectory_popup,eventdata,handles);



% --- Executes during object creation, after setting all properties.
function num_points_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_points_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in stretch_trajectory_popup.
function stretch_trajectory_popup_Callback(hObject, eventdata, handles)
global stretch_trajectory stretch_number
global humanoid_part
update_points(humanoid_part,handles);
update_support_foot(handles);
stretch_number = get(hObject,'Value');
messages_master(handles)
set(handles.humanoid_part_panel,'Title',['Poses - "Stretch ' stretch_trajectory{stretch_number} '"']);
show_selected_values(handles)


% --- Executes during object creation, after setting all properties.
function stretch_trajectory_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stretch_trajectory_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function add_default_data()
global trajectory_points total_points SETTINGS_TEO
actual_size=size(trajectory_points.initial_point.CoM,2);
for ii=1:(total_points-actual_size);    
    for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
        trajectory_points.initial_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end+1) = trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end);
        trajectory_points.final_point.(SETTINGS_TEO.humanoid_fields(jj).name)(:,end+1) = zeros(SETTINGS_TEO.humanoid_fields(jj).size, 1);
        trajectory_points.t0_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1) = 0;
        trajectory_points.T_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1) = 0;
        trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1) = SETTINGS_TEO.parameters.Ts;
        trajectory_points.pos_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
        trajectory_points.orient_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
        trajectory_points.time_diff.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Diff';
        trajectory_points.interpola_pos.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Linear';
        trajectory_points.interpola_orient.(SETTINGS_TEO.humanoid_fields(jj).name){end+1} = 'Linear';  
        trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(end+1) = 0;
    end
end

function update_support_foot(handles, update_last)
global trajectory_points SETTINGS_TEO stretch_number
if nargin < 2
  update_last = 0;
end
support_foot=get(handles.legs_popup,'Value');
% Double or Simple Support
% 0--> Double. -1--> Right Foot. 1-->Left Support
% All humanoid_parts have the same Support value
for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
    switch support_foot
        case 1
            trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(stretch_number)=0;
        case 2
            trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(stretch_number)=-1;
        case 3
            trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(stretch_number)=1;
    end
end

function messages_master(handles)
global humanoid_part stretch_number SETTINGS_TEO trajectory_points
if (strcmp(humanoid_part,'RF')&&(trajectory_points.support_foot.RF(stretch_number)==0))
    set(handles.label_points,'String','Right Foot will not move in Double Support');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]);
elseif (strcmp(humanoid_part,'RF')&&(trajectory_points.support_foot.RF(stretch_number)==-1))
    set(handles.label_points,'String','Right Foot will not move in Right Leg Support');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]);    
elseif (strcmp(humanoid_part,'LF')&&(trajectory_points.support_foot.RF(stretch_number)==0))
    set(handles.label_points,'String','Left Foot will not move in Double Support');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]);
elseif (strcmp(humanoid_part,'LF')&&(trajectory_points.support_foot.RF(stretch_number)==1))
    set(handles.label_points,'String','Left Foot will not move in Left Leg Support');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]);    
else
    %Clear label_points
    set(handles.label_points,'String','');
    set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
end


% --- Executes on button press in save_ik_button.
function save_ik_button_Callback(hObject, eventdata, handles)
global q dq ddq
global save_q save_dq save_ddq traj_name_string
traj_name_string = get(handles.traj_name_text,'String');
save_q = q;
save_dq = dq;
save_ddq = ddq;
save_trajectory_window;


% --- Executes on button press in visualize_axis_movement_pushbutton.
function visualize_axis_movement_pushbutton_Callback(hObject, eventdata, handles)
global trajectory humanoid_part SETTINGS_TEO
pose_rpy=(trajectory.(humanoid_part))';
pose_tr=zeros(4,4,size(pose_rpy,1));

for i=1:size(pose_rpy,1)
    pose_tr(:,:,i)=transl(pose_rpy(i,1:3))* rpy2tr(pose_rpy(i,4:6));
end
try
    set(handles.label_points,'String','');
    set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
    figure(60),title(['Humanoid Axes: ' humanoid_part '   ---   Ts = ' num2str(SETTINGS_TEO.parameters.Ts)]),tranimate(pose_tr); 
catch
    set(handles.label_points,'String','Visualize Axis Movement aborted!');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]); 
end


% --- Executes on button press in visualize_humanoid_movement_pushbutton.
function visualize_humanoid_movement_pushbutton_Callback(hObject, eventdata, handles)
global trajectory q SETTINGS_TEO humanoid_part
try
    set(handles.label_points,'String','');
    set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
    TEO_Plot_function (trajectory, q, SETTINGS_TEO.TEO, SETTINGS_TEO.h);
catch
    set(handles.label_points,'String','Visualize Humanoid Movement aborted!');
    set(handles.label_points,'BackGroundColor',[1 1 0.2]);     
end
