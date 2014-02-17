function varargout = main_TEO(varargin)
% main_TEO MATLAB code for main_TEO.fig
%      main_TEO, by itself, creates a new main_TEO or raises the existing
%      singleton*.
%
%      H = main_TEO returns the handle to a new main_TEO or the handle to
%      the existing singleton*.
%
%      main_TEO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in main_TEO.M with the given input arguments.
%
%      main_TEO('Property','Value',...) creates a new main_TEO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before main_TEO_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to main_TEO_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help main_TEO

% Last Modified by GUIDE v2.5 11-Sep-2013 20:36:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @main_TEO_OpeningFcn, ...
                   'gui_OutputFcn',  @main_TEO_OutputFcn, ...
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


% --- Executes just before main_TEO is made visible.
function main_TEO_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
%--------------------------------------------------------------------------
% To put screen correctly

scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);

% to chargue main image 'portada.jpg'
axes(handles.axes1)
[r,map]=imread('portada.jpg','jpg');
image(r);colormap(map);axis off
axes(handles.axes2)
[r,map]=imread('uc3m.png','png');
image(r);colormap(map);axis off
% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = main_TEO_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

% --- Executes on button press in button_trajectory_generation.
function button_trajectory_generation_Callback(hObject, eventdata, handles)

h = waitbar(0,'Please wait...');
steps = 300;

for step = 1:steps
    % computations take place here
    waitbar(step / steps)
end

settings_trajectory_generation
close(h)
close main_TEO

% --- Executes on button press in steps_control_button.
function steps_control_button_Callback(hObject, eventdata, handles)

steps_control

close main_TEO
