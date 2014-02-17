function varargout = design_step(varargin)
% DESIGN_STEP GUI
%
%   This GUI allows user edit the variation of CoM, Right Foot, Left Foot,
%   Right Arm and Left Arm. The user can also choose the type of
%   interpolation.
%
%   Author: Domingo Esteban
%   References from: P. Pierro and D. Garc�a-Cano
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/08/05 $

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @design_step_OpeningFcn, ...
                   'gui_OutputFcn',  @design_step_OutputFcn, ...
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


% --- Executes just before design_step is made visible.
function design_step_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to design_step (see VARARGIN)
% Choose default command line output for design_step
global h TEO pvia

% Hide plot_options panel
set(handles.plot_options, 'Visible', 'Off')

% Choose default command line output for design_step
handles.output = hObject;

handles.interpola_DS='Linear';
handles.interpola_SS_com='Linear';
handles.interpola_SS_ff='Linear';
handles.interpola_RH='Linear';
handles.interpola_LH='Linear';

if isempty(varargin)
    % Default values
    handles.Input_data.Ts_val=0.02;
    handles.Input_data.T_val=4;  
    handles.Input_data.alpha_ds=1/3;
    handles.Input_data.alpha_sf=0.2;
    handles.Input_data.DS_or_SS='Double and Simple';
    handles.Input_data.Leg='Right Leg';
    handles.Input_data.L_val=0.05;
    handles.Input_data.H_val=0.03;
%     handles.Input_data.q0=[0; 0.00325683448936741; -0.308647699300050;...
%             0.796421295515307; -0.487773596215257; 0.0278918646012491;...
%             0; 0.00325683448936741; -0.311486990906165;...
%             0.796421295515307;...
%             -0.484850796032492; -0.0354911450764397; 0.0349065850398866;...
%             1.57079632679490; -0.167017153300893; 0; -0.734875474523928;...
%             0; 1.57079632679490; 0.167017153300893; 0;...
%              -0.734875474523928; 0;0;0;0];
   handles.Input_data.q0=[0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;... % Right Leg
     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...                     % Left Leg
     0.0349065850398866; 0;...                                                                                                      % Waist
     1.57079632679490; -0.167017153300893; 0; -0.734875474523928; 0; 0;                                                             % Right Arm
     1.57079632679490; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                                            % Left Arm   
%     handles.Input_data.q0=ones(26,1)*0.000001;

else
    handles.Input_data = varargin{:};
end

% Assign Input Data
    L = handles.Input_data.L_val;
    H = handles.Input_data.H_val;
    q0 = handles.Input_data.q0;
    Leg = handles.Input_data.Leg;
    %set(handles.text1_design,'String',Leg);
    DS_or_SS = handles.Input_data.DS_or_SS;
    alpha_sf = handles.Input_data.alpha_sf;

% Load TEO Kinematics Library and Structure
h = TEO_kinematics_library;
TEO = TEO_structure('numeric', 'rad', 'm'); 
handles.humanoid_structure=TEO;

if strcmp(DS_or_SS,'Simple')
    %set(handles.double_support_data_panel,'Visible','off');
    set(handles.delta_x_CoM_DS,'Enable','inactive');
    set(handles.delta_y_CoM_DS,'Enable','inactive');
    set(handles.delta_z_CoM_DS,'Enable','inactive');
    set(handles.interpolaDS,'Enable','inactive');
    %children = get(handles.uipanelDelta_CoM,'Children');
    %children = get(handles.double_support_data_panel,'Children');
    %set(children,'Enable','inactive');
    %children = get(handles.uipanelDS_interpolation,'Children');
    %set(children,'Enable','inactive');
    %set(handles.text2_design,'String','No Double Support');
    set(handles.double_support_data_panel,'Title','No Double Support');
else
    set(handles.double_support_data_panel,'Visible','on');
end


% Pre-evaluate delta x and delta y CoM in Double Support
switch Leg
    case 'Right Leg' % Support on right foot
        delta = h.CoM_T_RF(q0);
        delta_P(1,:) = (2*delta(1) + (TEO.legs.right.foot.limits.x(1) + TEO.legs.right.foot.limits.x(2))/1);
        delta_P(2,:) = (2*delta(2) + (TEO.legs.right.foot.limits.y(1) + TEO.legs.right.foot.limits.y(2))/1);
        
    case 'Left Leg' % Support on left foot
        delta = h.CoM_T_LF(q0);
        delta_P(1,:) = (2*delta(1) + (TEO.legs.left.foot.limits.x(1) + TEO.legs.left.foot.limits.x(2))/1);
        delta_P(2,:) = (2*delta(2) + (TEO.legs.left.foot.limits.y(1) + TEO.legs.left.foot.limits.y(2))/1);
        
end
delta_P = delta_P/2;
set(handles.delta_x_CoM_DS,'String',num2str(delta_P(1)),'BackgroundColor','red');
set(handles.delta_y_CoM_DS,'String',num2str(delta_P(2)),'BackgroundColor','red');

X1_SF = L*alpha_sf/2;
set(handles.delta_x_CoM_SS1,'String',num2str(X1_SF),'BackgroundColor','red');
X2_SF = L*alpha_sf;
set(handles.delta_x_CoM_SS2,'String',num2str(X2_SF),'BackgroundColor','red');

X1_FF = L*(1-alpha_sf)/2;
set(handles.delta_x_FF_SS1,'String',num2str(X1_FF),'BackgroundColor','red');
set(handles.delta_z_FF_SS1,'String',num2str(H),'BackgroundColor','red');

X2_FF = L*(1-alpha_sf);
set(handles.delta_x_FF_SS2,'String',num2str(X2_FF),'BackgroundColor','red');

pvia = zeros(3,1);

graphstab = uitabpanel(...
  'Parent',handles.panel_graphics,...
  'Style','popup',...
  'Units','normalized',...
  'Position',[0,0,1,1],...
  'FrameBackgroundColor',[0.078,0.169,0.549],...
  'FrameBorderType','etchedin',...
  'Title',{'Right Leg','Left Leg','Torso','Right Arm','Left Arm','CoM'},...
  'PanelHeights',[59.5,59.5,50,59.5,59.5,50],...
  'HorizontalAlignment','left',...
  'FontWeight','bold',...
  'TitleBackgroundColor',[0.078,0.169,0.549],...
  'TitleForegroundColor',[1 1 1],...
  'PanelBackgroundColor',[0.702,0.78,1],...
  'PanelBorderType','line','SelectedItem',6);

hpanel = getappdata(graphstab,'panels');



% *******************************

   handles.sd_graph = axes('Parent',getappdata(graphstab,'status'),...
    'Units','normalized',...
      'Position',[0.3,0.1,0.4,0.9],'Box','on'); 
  
   % Ejes para mostrar el valor de las articulaciones de la pierna derecha
   handles.axes1 = axes('Parent',hpanel(1),'Position',[.1 .6 .25 .25]);
   handles.axes2 = axes('Parent',hpanel(1),'Position',[.4 .6 .25 .25]);
   handles.axes3 = axes('Parent',hpanel(1),'Position',[.7 .6 .25 .25]);
   handles.axes4 = axes('Parent',hpanel(1),'Position',[.1 .15 .25 .25]);
   handles.axes5 = axes('Parent',hpanel(1),'Position',[.4 .15 .25 .25]);
   handles.axes6 = axes('Parent',hpanel(1),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones de la pierna
   % izquierda
   handles.axes7 = axes('Parent',hpanel(2),'Position',[.1 .6 .25 .25]);
   handles.axes8 = axes('Parent',hpanel(2),'Position',[.4 .6 .25 .25]);
   handles.axes9 = axes('Parent',hpanel(2),'Position',[.7 .6 .25 .25]);
   handles.axes10 = axes('Parent',hpanel(2),'Position',[.1 .15 .25 .25]);
   handles.axes11 = axes('Parent',hpanel(2),'Position',[.4 .15 .25 .25]);
   handles.axes12 = axes('Parent',hpanel(2),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones del torso
   handles.axes13 = axes('Parent',hpanel(3),'Position',[.1 .6 .25 .25]);
   handles.axes14 = axes('Parent',hpanel(3),'Position',[.7 .6 .25 .25]);   
   
   % Ejes para mostrar el valor de las articulaciones del brazo derecho
   handles.axes15 = axes('Parent',hpanel(4),'Position',[.1 .6 .25 .25]);
   handles.axes16 = axes('Parent',hpanel(4),'Position',[.4 .6 .25 .25]);
   handles.axes17 = axes('Parent',hpanel(4),'Position',[.7 .6 .25 .25]);
   handles.axes18 = axes('Parent',hpanel(4),'Position',[.1 .15 .25 .25]);
   handles.axes19 = axes('Parent',hpanel(4),'Position',[.4 .15 .25 .25]);
   handles.axes20 = axes('Parent',hpanel(4),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones del brazo izquierdo
   handles.axes21 = axes('Parent',hpanel(5),'Position',[.1 .6 .25 .25]);
   handles.axes22 = axes('Parent',hpanel(5),'Position',[.4 .6 .25 .25]);
   handles.axes23 = axes('Parent',hpanel(5),'Position',[.7 .6 .25 .25]);
   handles.axes24 = axes('Parent',hpanel(5),'Position',[.1 .15 .25 .25]);
   handles.axes25 = axes('Parent',hpanel(5),'Position',[.4 .15 .25 .25]);
   handles.axes26 = axes('Parent',hpanel(5),'Position',[.7 .15 .25 .25]);

   
   % Ejes para mostrar el valor del CoM
   handles.axes_zmp = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
   %handles.axes_robot_plot = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
   
   % Ejes para mostrar la imagen de TEO
   axes(handles.sd_graph)
   [r,map]=imread('portada.jpg','jpg');
   image(r);colormap(map);axis off
   
   axes(handles.axes_zmp)
   [r,map]=imread('portada.jpg','jpg');
   image(r);colormap(map);axis off

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes design_step wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = design_step_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in interpolaDS.
function interpolaDS_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_DS = contents{get(hObject,'Value')};
guidata(hObject,handles)

% Hints: contents = cellstr(get(hObject,'String')) returns interpolaDS contents as cell array
%        contents{get(hObject,'Value')} returns selected item from interpolaDS


% --- Executes during object creation, after setting all properties.
function interpolaDS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpolaDS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_CoM_DS_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_CoM_DS as text
%        str2double(get(hObject,'String')) returns contents of delta_x_CoM_DS as a double


% --- Executes during object creation, after setting all properties.
function delta_x_CoM_DS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_CoM_DS_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_CoM_DS as text
%        str2double(get(hObject,'String')) returns contents of delta_y_CoM_DS as a double


% --- Executes during object creation, after setting all properties.
function delta_y_CoM_DS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_CoM_DS_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_CoM_DS as text
%        str2double(get(hObject,'String')) returns contents of delta_z_CoM_DS as a double


% --- Executes during object creation, after setting all properties.
function delta_z_CoM_DS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_DS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbuttonGenerateStep_Callback(hObject, eventdata, handles)
global q trajectory d_trajectory dd_trajectory
% Steps Data
    global data
    data.TS = handles.Input_data.Ts_val;            % Sampling Time
    data.t0 = 0;                                    % Initial time
    data.T = handles.Input_data.T_val;              % Total time. Time of the step
    data.alpha_ds = handles.Input_data.alpha_ds;    % Percentage of the total time for double support
    data.alpha_sf = handles.Input_data.alpha_sf;    % Percentage of the total time for support foot ???
    data.DS_or_SS = handles.Input_data.DS_or_SS;    %
    data.L = handles.Input_data.L_val;              % Length of the step (X direction)
    data.H = handles.Input_data.H_val;              % Height of the step (Z direction)
    data.q0 = handles.Input_data.q0;                % Initial pose

    
% Delta Data
global delta
    delta.delta_CoM_DS = [str2double(get(handles.delta_x_CoM_DS,'String'));...      % Variation of the CoM in the Double Support phase
        str2double(get(handles.delta_y_CoM_DS,'String'));...
        str2double(get(handles.delta_z_CoM_DS,'String'));zeros(3,1)];
    delta.interpola_CoM_DS = handles.interpola_DS;                                  % Interpolation for the CoM in the Double Support phase
    
    delta.delta_CoM_SS1 = [str2double(get(handles.delta_x_CoM_SS1,'String'));...    % Variation of the CoM in the Simple Support phase 1
        str2double(get(handles.delta_y_CoM_SS1,'String'));...
        str2double(get(handles.delta_z_CoM_SS1,'String'));zeros(3,1)];
    delta.delta_CoM_SS2 = [str2double(get(handles.delta_x_CoM_SS2,'String'));...    % Variation of the CoM in the Simple Support phase 2
        str2double(get(handles.delta_y_CoM_SS2,'String'));...
        str2double(get(handles.delta_z_CoM_SS2,'String'));zeros(3,1)];
    delta.interpola_CoM_SS = handles.interpola_SS_com;                              % Interpolation for the CoM in the Simple Support phase (1 and 2)

    delta.delta_FF_SS1 = [str2double(get(handles.delta_x_FF_SS1,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
        str2double(get(handles.delta_y_FF_SS1,'String'));...
        str2double(get(handles.delta_z_FF_SS1,'String'));zeros(3,1)];
    delta.delta_FF_SS2 = [str2double(get(handles.delta_x_FF_SS2,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
        str2double(get(handles.delta_y_FF_SS2,'String'));...
        str2double(get(handles.delta_z_FF_SS2,'String'));zeros(3,1)];
    delta.interpola_FF_SS = handles.interpola_SS_ff;                                % Interpolation for the Floating Foot in the Simple Support phase (1 and 2)


    delta.delta_RH = [str2double(get(handles.delta_x_RH,'String'));...              % Variation of the Right Hand in the DS and SS phase
        str2double(get(handles.delta_y_RH,'String'));...
        str2double(get(handles.delta_z_RH,'String'));zeros(3,1)];
    delta.interpola_RH = handles.interpola_RH;                                      % Interpolation for the Right Hand in the DS and SS phase

    delta.delta_LH = [str2double(get(handles.delta_x_LH,'String'));...              % Variation of the Left Hand in the DS and SS phase
        str2double(get(handles.delta_y_LH,'String'));...
        str2double(get(handles.delta_z_LH,'String'));zeros(3,1)];
    delta.interpola_LH = handles.interpola_LH;                                      % Interpolation for the Left Hand in the DS and SS phase

    
% Wait bar
    waitbar1= waitbar(0,'Please wait...');
    steps = 400;
    for step = 1:steps
        waitbar(step / steps)
    end

% Double Step and Simple Step for TEO Robot
[q,dq,ddq,trajectory,d_trajectory,dd_trajectory] = ds_ss_step_TEO(delta,data,handles.Input_data.Leg);


%%%DOMINGO
% borrar el grafico actual de axes_zmp (imagen de TEO):
cla(handles.axes_zmp)

handles.result.q = q;
handles.result.dq = dq;
handles.result.ddq = ddq;
handles.result.trajectory = trajectory;
handles.result.d_trajectory = d_trajectory;
handles.result.dd_trajectory = dd_trajectory;

%Close wait bar when ds_ss_step_TEO has finished
    close(waitbar1)

%%%DOMINGO
% Inhabilitado:
% set(handles.axes_zmp,'NextPlot','Add'); 
% axis on
% newplot(handles.axes_zmp);

%prueba_ZMP(handles)

%Graficar valores de q en los ejes del panel
%plot_joint_values(handles);

%Mostrar panel de gr�ficos
set(handles.plot_options, 'Visible', 'On')
plot_all_graphs(hObject,handles,'PLOT SPACE') %Mostrar el valor de las articulaciones de forma predefinida

guidata(hObject,handles)


% --- Executes on selection change in interpolaSSff.
function interpolaSSff_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_SS_ff = contents{get(hObject,'Value')};
guidata(hObject,handles)

% Hints: contents = cellstr(get(hObject,'String')) returns interpolaSSff contents as cell array
%        contents{get(hObject,'Value')} returns selected item from interpolaSSff


% --- Executes during object creation, after setting all properties.
function interpolaSSff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpolaSSff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_FF_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_FF_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_x_FF_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_x_FF_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_FF_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_FF_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_y_FF_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_y_FF_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_FF_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_FF_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_z_FF_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_z_FF_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_FF_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_CoM_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_CoM_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_x_CoM_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_x_CoM_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_CoM_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_CoM_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_y_CoM_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_y_CoM_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_CoM_SS2_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_CoM_SS2 as text
%        str2double(get(hObject,'String')) returns contents of delta_z_CoM_SS2 as a double


% --- Executes during object creation, after setting all properties.
function delta_z_CoM_SS2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_SS2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_FF_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_FF_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_x_FF_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_x_FF_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_FF_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_FF_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_y_FF_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_y_FF_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_FF_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_FF_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_z_FF_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_z_FF_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_FF_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in interpolaSScom.
function interpolaSScom_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_SS_com = contents{get(hObject,'Value')};
guidata(hObject,handles)

% Hints: contents = cellstr(get(hObject,'String')) returns interpolaSScom contents as cell array
%        contents{get(hObject,'Value')} returns selected item from interpolaSScom


% --- Executes during object creation, after setting all properties.
function interpolaSScom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpolaSScom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_CoM_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_CoM_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_x_CoM_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_x_CoM_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_CoM_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_CoM_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_y_CoM_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_y_CoM_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_CoM_SS1_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_CoM_SS1 as text
%        str2double(get(hObject,'String')) returns contents of delta_z_CoM_SS1 as a double


% --- Executes during object creation, after setting all properties.
function delta_z_CoM_SS1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_CoM_SS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_RH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_RH as text
%        str2double(get(hObject,'String')) returns contents of delta_x_RH as a double


% --- Executes during object creation, after setting all properties.
function delta_x_RH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_RH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_RH as text
%        str2double(get(hObject,'String')) returns contents of delta_y_RH as a double


% --- Executes during object creation, after setting all properties.
function delta_y_RH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_RH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_RH as text
%        str2double(get(hObject,'String')) returns contents of delta_z_RH as a double


% --- Executes during object creation, after setting all properties.
function delta_z_RH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_RH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in interpolaLH.
function interpolaLH_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_LH = contents{get(hObject,'Value')};
guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function interpolaLH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to interpolaLH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_x_LH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_LH as text
%        str2double(get(hObject,'String')) returns contents of delta_x_LH as a double


% --- Executes during object creation, after setting all properties.
function delta_x_LH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_y_LH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_LH as text
%        str2double(get(hObject,'String')) returns contents of delta_y_LH as a double


% --- Executes during object creation, after setting all properties.
function delta_y_LH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_z_LH_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_LH as text
%        str2double(get(hObject,'String')) returns contents of delta_z_LH as a double


% --- Executes during object creation, after setting all properties.
function delta_z_LH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_LH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function interpolaRH_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_RH = contents{get(hObject,'Value')};

guidata(hObject,handles)


function interpolaRH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in plot_options.
function plot_options_SelectionChangeFcn(hObject, eventdata, handles)
handles.plot_new = get(eventdata.NewValue,'String');
old_val = get(eventdata.OldValue,'String');

for jj =1:6
            cla(handles.(strcat('axes',num2str(jj))))
            cla(handles.(strcat('axes',num2str(jj+6))))
            

end
for jj=1:5
            cla(handles.(strcat('axes',num2str(jj+13))))
            cla(handles.(strcat('axes',num2str(jj+18)))) 
end

plot_all_graphs(hObject,handles,handles.plot_new)

guidata(hObject,handles)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in button_TEO_movement.
function button_TEO_movement_Callback(hObject, eventdata, handles)
global trajectory q TEO h
TEO_Plot_function(trajectory, q, TEO, h)


% --------------------------------------------------------------------
function menu_steps_control_Callback(hObject, eventdata, handles)
steps_control
guidata(hObject,handles)
close(handles.figure1)


% --------------------------------------------------------------------
function menu_main_TEO_Callback(hObject, eventdata, handles)
main_TEO
guidata(hObject,handles)
close(handles.figure1)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function ang_vel_acc_save_txt_menu_Callback(hObject, eventdata, handles)
Q = handles.result.q;
dQ = handles.result.dq;
ddQ = handles.result.ddq;
[m,n] = size(Q);
%Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
%dQtext = [dQ(1:6,:);dQ(14:17,:);dQ(7:12,:);dQ(19:22,:);dQ(13,:);zeros(1,n);dQ(18,:)];
%ddQtext = [ddQ(1:6,:);ddQ(14:17,:);ddQ(7:12,:);ddQ(19:22,:);ddQ(13,:);zeros(1,n);ddQ(18,:)];
try
    [file1,path] = uiputfile('*.txt','Save Joint Angles as');
    dlmwrite(file1,Q,'delimiter','\t','precision','%.6f')
catch
    disp('Save joint angles *.txt aborted');
end
try
    [file2,path] = uiputfile('*.txt','Save Joint Velocities as');
    dlmwrite(file2,dQ,'delimiter','\t','precision','%.6f')
catch
    disp('Save joint velocities *.txt aborted');
end
try
    [file3,path] = uiputfile('*.txt','Save Joint Accelerations as');
    dlmwrite(file3,ddQ,'delimiter','\t','precision','%.6f')
catch
    disp('Save joint acceleration *.txt aborted');
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function ang_vel_acc_save_csv_menu_Callback(hObject, eventdata, handles)
Q = handles.result.q;
dQ = handles.result.dq;
ddQ = handles.result.ddq;
[m,n] = size(Q);
%Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
%dQtext = [dQ(1:6,:);dQ(14:17,:);dQ(7:12,:);dQ(19:22,:);dQ(13,:);zeros(1,n);dQ(18,:)];
%ddQtext = [ddQ(1:6,:);ddQ(14:17,:);ddQ(7:12,:);ddQ(19:22,:);ddQ(13,:);zeros(1,n);ddQ(18,:)];
try
[file1,path] = uiputfile('*.csv','Save Joint Angles as');
csvid = fopen(file1, 'w');
fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', Q');
fclose(csvid);
catch
    disp('Save joint angles *.csv aborted');
end

try
    [file2,path] = uiputfile('*.csv','Save Joint Velocities as');
    csvid = fopen(file2, 'w');
    fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', dQ');
    fclose(csvid);
catch
    disp('Save joint velocities *.csv aborted');
end

try
    [file3,path] = uiputfile('*.csv','Save Joint Accelerations as');
    csvid = fopen(file3, 'w');
    fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', ddQ');
    fclose(csvid);
catch
    disp('Save joint accelerations *.csv aborted');
end


% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function angles_save_txt_menu_Callback(hObject, eventdata, handles)
%
Q = handles.result.q;
[m,n] = size(Q);
%Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
try
[file,path] = uiputfile('*.txt','Save Joint Angles as');
dlmwrite(file,Q,'delimiter','\t','precision','%.6f')
catch
    disp('Save joint angles *.txt aborted');
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function angles_save_csv_menu_Callback(hObject, eventdata, handles)

Q = handles.result.q;
[m,n] = size(Q);
%Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
try
[file,path] = uiputfile('*.csv','Save Joint Angles as');
csvid = fopen(file, 'w');
fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', Q);
fclose(csvid);
catch
    disp('Save joint angles *.csv aborted');
end
guidata(hObject,handles)
