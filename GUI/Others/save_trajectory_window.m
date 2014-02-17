function varargout = save_trajectory_window(varargin)
% SAVE_TRAJECTORY_WINDOW MATLAB code
% Author: Domingo Esteban
%
% RobitcsLab, Universidad Carlos III de Madrid
% 2013

% Edit the above text to modify the response to help save_trajectory_window

% Last Modified by GUIDE v2.5 18-Sep-2013 19:24:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @save_trajectory_window_OpeningFcn, ...
                   'gui_OutputFcn',  @save_trajectory_window_OutputFcn, ...
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


% --- Executes just before save_trajectory_window is made visible.
function save_trajectory_window_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for save_trajectory_window
handles.output = hObject;

movegui(gcf,'center')

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes save_trajectory_window wait for user response (see UIRESUME)
% uiwait(handles.save_format);


% --- Outputs from this function are returned to the command line.
function varargout = save_trajectory_window_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on selection change in format_popup.
function format_popup_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function format_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to format_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_pushbutton.
function save_pushbutton_Callback(hObject, eventdata, handles)
option=get(handles.format_popup,'Value');
global save_trajectory save_d_trajectory save_dd_trajectory traj_name_string
global q dq ddq
try
    if option==1
        [m,n] = size(q);
        [file1,path] = uiputfile([traj_name_string '_q.csv'],'Save Joint Angles as');
        csvid = fopen(file1, 'w');
        fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', q');
        fclose(csvid);

        [file2,path] = uiputfile([traj_name_string '_dq.csv'],'Save Joint Velocities as');
        csvid = fopen(file2, 'w');
        fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', dq');
        fclose(csvid);

        [file3,path] = uiputfile([traj_name_string '_ddq.csv'],'Save Joint Accelerations as');
        csvid = fopen(file3, 'w');
        fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', ddq');
        fclose(csvid);

    elseif option==2
        save{1}='trajectory';
        save{2}='d_trajectory';
        save{3}='dd_trajectory';
        for i=1:3
            Q = eval(['save_' (save{i})]);
            [m,n] = size(Q);
            %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
            [file,path] = uiputfile([traj_name_string '_' save{i} '.txt'],['Save ' save{i} ' as:']);
            dlmwrite(file,Q,'delimiter','\t','precision','%.6f')
        end
    end
catch
    disp('Save trajectories/joints canceled');
end
close(save_trajectory_window)
