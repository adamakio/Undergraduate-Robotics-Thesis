function varargout = mytry1(varargin)
global a1 a2
% MYTRY1 M-file for mytry1.fig
%      MYTRY1, by itself, creates a new MYTRY1 or raises the existing
%      singleton*.
%
%      H = MYTRY1 returns the handle to a new MYTRY1 or the handle to
%      the existing singleton*.
%
%      MYTRY1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYTRY1.M with the given input arguments.
%
%      MYTRY1('Property','Value',...) creates a new MYTRY1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before testGui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mytry1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mytry1

% Last Modified by GUIDE v2.5 13-Nov-2008 04:19:55

% Begin initialization code - DO NOT EDIT
a1=4; a2=3;
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mytry1_OpeningFcn, ...
                   'gui_OutputFcn',  @mytry1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before mytry1 is made visible.
function mytry1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mytry1 (see VARARGIN)

% Choose default command line output for mytry1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using mytry1.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
    cla;
    plotbot2(0,0,0,0);
end
h = gcf;
set(h,'Color',[0.584 0.902 0.918]);


% UIWAIT makes mytry1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mytry1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)

% --- Executes during object creation, after setting all properties.
function x_position_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function x_position_Callback(hObject, eventdata, handles)
% hObject    handle to x_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_position as text
%        str2double(get(hObject,'String')) returns contents of x_position as a double
x = round(str2double(get(hObject,'String'))*10)/10;
set(handles.x_position,'String',x);
set(handles.x_slider,'Value',x);


% --- Executes during object creation, after setting all properties.
function y_position_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function y_position_Callback(hObject, eventdata, handles)
% hObject    handle to y_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_position as text
%        str2double(get(hObject,'String')) returns contents of y_position as a double
y = round(str2double(get(hObject,'String'))*10)/10;
set(handles.y_position,'String',y);
set(handles.y_slider,'Value',y);

% --- Executes during object creation, after setting all properties.
function theta1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function theta1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
theta1 = round(get(hObject,'Value')*10)/10;
set(handles.theta1_slider,'Value',theta1);
set(handles.theta1_angle,'String',theta1);


% --- Executes during object creation, after setting all properties.
function theta1_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function theta1_angle_Callback(hObject, eventdata, handles)
% hObject    handle to theta1_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1_angle as text
%        str2double(get(hObject,'String')) returns contents of theta1_angle as a double
theta1 = round(str2double(get(hObject,'String'))*10)/10;
set(handles.theta1_angle,'String',theta1);
set(handles.theta1_slider,'Value',theta1);


% --- Executes during object creation, after setting all properties.
function theta2_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function theta2_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
theta2 = round(get(hObject,'Value')*10)/10;
set(handles.theta2_slider,'Value',theta2);
set(handles.theta2_angle,'String',theta2);


% --- Executes during object creation, after setting all properties.
function theta2_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function theta2_angle_Callback(hObject, eventdata, handles)
% hObject    handle to theta2_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2_angle as text
%        str2double(get(hObject,'String')) returns contents of theta2_angle as a double
theta2 = round(str2double(get(hObject,'String'))*10)/10;
set(handles.theta2_angle,'String',theta2);
set(handles.theta2_slider,'Value',theta2);


% --- Executes during object creation, after setting all properties.
function xe_position_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xe_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function xe_position_Callback(hObject, eventdata, handles)
% hObject    handle to xe_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xe_position as text
%        str2double(get(hObject,'String')) returns contents of xe_position as a double
xe = round(str2double(get(hObject,'String'))*10)/10;
set(handles.xe_position,'String',xe);
set(handles.xe_slider,'Value',xe);

% --- Executes during object creation, after setting all properties.
function ye_position_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ye_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function ye_position_Callback(hObject, eventdata, handles)
% hObject    handle to ye_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ye_position as text
%        str2double(get(hObject,'String')) returns contents of ye_position as a double
ye = round(str2double(get(hObject,'String'))*10)/10;
set(handles.ye_position,'String',ye);
set(handles.ye_slider,'Value',ye);

% --- Executes on button press in inv_update.
function inv_update_Callback(hObject, eventdata, handles)
% hObject    handle to inv_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[xe,ye] = getEE(handles);
[r1,r2,r4,r5] = getConfig(handles);
r = [r1 r2 r4 r5]';
[x,y,th1,th2] = botInvKinematics2(xe,ye,r);
axes(handles.axes1);
cla;
plotbot2(x,y,th1,th2);
updateConfig(handles,x,y,th1,th2);
[xe,ye] = botFwdKinematics2(x,y,th1,th2);
updateEE(handles,xe,ye);

% --- Executes on button press in fwd_update.
function fwd_update_Callback(hObject, eventdata, handles)
% hObject    handle to fwd_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y,th1,th2] = getConfig(handles);
axes(handles.axes1);
cla;
plotbot2(x,y,th1,th2);
[xe,ye] = botFwdKinematics2(x,y,th1,th2);
updateEE(handles,xe,ye);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                           %
%           Additional functions I wrote (begin)            %
%                                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function updateConfig(handles,x,y,theta1,theta2)
set(handles.x_position,'String',x);
set(handles.y_position,'String',y);
set(handles.theta1_slider,'Value',theta1*180/pi);
set(handles.theta1_angle,'String',theta1*180/pi);
set(handles.theta2_slider,'Value',theta2*180/pi);
set(handles.theta2_angle,'String',theta2*180/pi);

function updateEE(handles,xe,ye)
set(handles.xe_position,'String',xe);
set(handles.ye_position,'String',ye);
set(handles.xe_slider,'Value',xe);
set(handles.ye_slider,'Value',ye);

function [x,y,theta1,theta2] = getConfig(handles)
x = str2double(get(handles.x_position,'String'));
y = str2double(get(handles.y_position,'String'));
theta1 = str2double(get(handles.theta1_angle,'String'))*pi/180;
theta2 = str2double(get(handles.theta2_angle,'String'))*pi/180;

function [xe,ye] = getEE(handles)
xe = str2double(get(handles.xe_position,'String'));
ye = str2double(get(handles.ye_position,'String'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                           %
%           Additional functions I wrote (end)              %
%                                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes during object creation, after setting all properties.

% --- Executes on slider movement.
% --- Executes during object creation, after setting all properties.
function x_radius_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function x_radius_slider_Callback(hObject, eventdata, handles)
% hObject    handle to x_radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
r = round(get(hObject,'Value')*10)/10;
set(handles.x_radius_slider,'Value',r);
set(handles.x_radius,'String',r);

function y_radius_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function y_radius_slider_Callback(hObject, eventdata, handles)
% hObject    handle to y_radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
r = round(get(hObject,'Value')*10)/10;
set(handles.y_radius_slider,'Value',r);
set(handles.y_radius,'String',r);

% --- Executes on button press in trackCircle.
function trackCircle_Callback(hObject, eventdata, handles)
% hObject    handle to trackCircle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a1 a2
d2r=3.1416/180;
ell_an = str2double(get(handles.ellipse_angle,'String'))*d2r; %converted to radians
rad_x = str2double(get(handles.x_radius,'String'));
rad_y = str2double(get(handles.y_radius,'String'));
xc = str2double(get(handles.xCenter,'String'));
yc = str2double(get(handles.yCenter,'String'));
x = xc + rad_x*cos(ell_an) - (a1+a2);
y = yc + rad_x*sin(ell_an);
th1 = 0;
th2 = 0;
[xe,ye] = botFwdKinematics2(x,y,th1,th2);
axes(handles.axes1);
cla;
plotbot2(x,y,th1,th2);
pause(0.01);
updateConfig(handles,x,y,th1,th2);
updateEE(handles,xe,ye);

simTime = str2double(get(handles.simTime,'String'));
Trev = str2double(get(handles.revTime,'String'));
dt = 0.1;
t = 0:dt:simTime;
N = length(t);
Qo = str2double(get(handles.weightQo,'String'));
Qp = str2double(get(handles.weightQp,'String'));
Q = [eye(2)*Qp zeros(2,2); zeros(2,2) eye(2)*Qo];
r = [x y th1 th2]';
re = [xc + rad_x*cos(2*pi*t/Trev)*cos(ell_an) - rad_y*sin(2*pi*t/Trev)*sin(ell_an); ...
      yc + rad_x*cos(2*pi*t/Trev)*sin(ell_an) + rad_y*sin(2*pi*t/Trev)*cos(ell_an)];
ra = zeros(2,N);
rb = zeros(2,N);

for k = 1:N-1,
    P = eye(2);
    for j = 1:10,
    tc = dt/10*(j-1) + t(k);
    rDot = [-2*pi*rad_x*sin(2*pi*tc/Trev)/Trev*cos(ell_an)-2*pi*rad_y*cos(2*pi*tc/Trev)/Trev*sin(ell_an); ...
            -2*pi*rad_x*sin(2*pi*tc/Trev)/Trev*sin(ell_an)+2*pi*rad_y*cos(2*pi*tc/Trev)/Trev*cos(ell_an)];
    f1 = dt/10*botThetaDot2(r, rDot, P, Q);
    f2 = dt/10*botThetaDot2(r+f1*0.5, rDot, P, Q);
    f3 = dt/10*botThetaDot2(r+f2*0.5, rDot, P, Q);
    f4 = dt/10*botThetaDot2(r+f3, rDot, P, Q);
    r = r + (f1+2*f2+2*f3+f4)/6;
    end
    x = r(1); y = r(2); th1 = r(3); th2 = r(4);
    rb(:,k) = [x y]';
    while (2*pi<th1)||(0>th1)
        th1 = th1 - sign(th1)*2*pi;
    end
    while (2*pi<th2)||(0>th2)
        th2 = th2 - sign(th2)*2*pi;
    end
    [xe,ye] = botFwdKinematics2(x,y,th1,th2);
    ra(:,k) = [xe ye]';
    cla;
    plot(re(1,:),re(2,:),'b'); hold on;
    plot(ra(1,1:k),ra(2,1:k),'r');
    plot(rb(1,1:k),rb(2,1:k),'k');
    plotbot2(x,y,th1,th2);
%    if rem(k,5)==0
    updateConfig(handles,x,y,th1,th2);
    updateEE(handles,xe,ye);
    %    end
    pause(0.01);
    flag = get(handles.stop,'Value');
    if flag == 1
        set(handles.stop,'Value',0);
        break;
    end
end

% --- Executes during object creation, after setting all properties.
function xCenter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xCenter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function xCenter_Callback(hObject, eventdata, handles)
% hObject    handle to xCenter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xCenter as text
%        str2double(get(hObject,'String')) returns contents of xCenter as a double
x = round(str2double(get(hObject,'String'))*10)/10;
set(handles.xCenter,'String',x);
set(handles.xCenter_slider,'Value',x);

% --- Executes during object creation, after setting all properties.
function yCenter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yCenter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function yCenter_Callback(hObject, eventdata, handles)
% hObject    handle to yCenter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yCenter as text
%        str2double(get(hObject,'String')) returns contents of yCenter as a double
y = round(str2double(get(hObject,'String'))*10)/10;
set(handles.yCenter,'String',y);
set(handles.yCenter_slider,'Value',y);


% --- Executes during object creation, after setting all properties.
function x_radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function x_radius_Callback(hObject, eventdata, handles)
% hObject    handle to x_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_radius as text
%        str2double(get(hObject,'String')) returns contents of x_radius as a double
r = (round(str2double(get(hObject,'String'))*10))/10;
set(handles.x_radius,'String',r);
set(handles.x_radius_slider,'Value',r);

% --- Executes during object creation, after setting all properties.
function y_radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function y_radius_Callback(hObject, eventdata, handles)
% hObject    handle to y_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_radius as text
%        str2double(get(hObject,'String')) returns contents of y_radius as a double
r = (round(str2double(get(hObject,'String'))*10))/10;
set(handles.y_radius,'String',r);
set(handles.y_radius_slider,'Value',r);

% --- Executes during object creation, after setting all properties.
function ellipse_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ellipse_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function ellipse_angle_Callback(hObject, eventdata, handles)
% hObject    handle to ellipse_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of ellipse_angle as text
%        str2double(get(hObject,'String')) returns contents of ellipse_angle as a double
ea = round(str2double(get(hObject,'String'))*10)/10;
set(handles.ellipse_angle,'String',ea);
set(handles.ellipse_angle_slider,'Value',ea);


% --- Executes during object creation, after setting all properties.
function ellipse_angle_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ellipse_angle_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function ellipse_angle_slider_Callback(hObject, eventdata, handles)
% hObject    handle to ellipse_angle_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
ea = round(get(hObject,'Value')*10)/10;
set(handles.ellipse_angle_slider,'Value',ea);
set(handles.ellipse_angle,'String',ea);


% --- Executes during object creation, after setting all properties.
function revTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to revTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function revTime_Callback(hObject, eventdata, handles)
% hObject    handle to revTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of revTime as text
%        str2double(get(hObject,'String')) returns contents of revTime as a double
t = round(str2double(get(hObject,'String'))*10)/10;
set(handles.revTime,'String',t);
set(handles.time_slider,'Value',t);

% --- Executes during object creation, after setting all properties.
function time_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function time_slider_Callback(hObject, eventdata, handles)
% hObject    handle to time_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
t = round(get(hObject,'Value')*10)/10;
set(handles.time_slider,'Value',t);
set(handles.revTime,'String',t);


% --- Executes during object creation, after setting all properties.
function xCenter_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xCenter_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function xCenter_slider_Callback(hObject, eventdata, handles)
% hObject    handle to xCenter_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
x = round(get(hObject,'Value')*10)/10;
set(handles.xCenter_slider,'Value',x);
set(handles.xCenter,'String',x);

% --- Executes during object creation, after setting all properties.
function yCenter_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yCenter_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function yCenter_slider_Callback(hObject, eventdata, handles)
% hObject    handle to yCenter_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
y = round(get(hObject,'Value')*10)/10;
set(handles.yCenter_slider,'Value',y);
set(handles.yCenter,'String',y);


% --- Executes during object creation, after setting all properties.
function Qo_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Qo_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function Qo_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Qo_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = round(get(hObject,'Value')*10)/10;
set(handles.Qo_slider,'Value',q);
set(handles.weightQo,'String',q);

% --- Executes during object creation, after setting all properties.
function weightQo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weightQo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function weightQo_Callback(hObject, eventdata, handles)
% hObject    handle to weightQo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weightQo as text
%        str2double(get(hObject,'String')) returns contents of weightQo as a double
q = round(str2double(get(hObject,'String'))*10)/10;
set(handles.weightQo,'String',q);
set(handles.Qo_slider,'Value',q);


% --- Executes during object creation, after setting all properties.
function Qp_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Qp_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function Qp_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Qp_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
q = round(get(hObject,'Value')*10)/10;
set(handles.Qp_slider,'Value',q);
set(handles.weightQp,'String',q);

% --- Executes during object creation, after setting all properties.
function weightQp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weightQp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function weightQp_Callback(hObject, eventdata, handles)
% hObject    handle to weightQp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weightQp as text
%        str2double(get(hObject,'String')) returns contents of weightQp as a double
q = round(str2double(get(hObject,'String'))*10)/10;
set(handles.weightQp,'String',q);
set(handles.Qp_slider,'Value',q);


% --- Executes during object creation, after setting all properties.
function simTime_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simTime_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function simTime_slider_Callback(hObject, eventdata, handles)
% hObject    handle to simTime_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
t = get(hObject,'Value');
t = round(t*10);
t = t/10;
set(handles.simTime_slider,'Value',t);
set(handles.simTime,'String',t);

% --- Executes during object creation, after setting all properties.
function simTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function simTime_Callback(hObject, eventdata, handles)
% hObject    handle to simTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simTime as text
%        str2double(get(hObject,'String')) returns contents of simTime as a double
t = str2double(get(hObject,'String'));
t = round(t*10);
t = t/10;
set(handles.simTime,'String',t);
set(handles.simTime_slider,'Value',t');


% --- Executes on button press in home.
function home_Callback(hObject, eventdata, handles)
% hObject    handle to home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

x = 0; y = 0; v = 0; th1 = 0; th2 = 0;
updateConfig(handles,x,y,th1,th2);
[xe,ye] = botFwdKinematics2(x,y,th1,th2);
updateEE(handles,xe,ye);
cla;
plotbot2(x,y,th1,th2);


% --- Executes during object creation, after setting all properties.
function y_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function y_slider_Callback(hObject, eventdata, handles)
% hObject    handle to y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
y = round(get(hObject,'Value')*10)/10;
set(handles.y_slider,'Value',y);
set(handles.y_position,'String',y);

% --- Executes during object creation, after setting all properties.
function x_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function x_slider_Callback(hObject, eventdata, handles)
% hObject    handle to x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
x = round(get(hObject,'Value')*10)/10;
set(handles.x_slider,'Value',x);
set(handles.x_position,'String',x);


% --- Executes during object creation, after setting all properties.
function ye_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ye_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function ye_slider_Callback(hObject, eventdata, handles)
% hObject    handle to ye_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
ye = round(get(hObject,'Value')*10)/10;
set(handles.ye_slider,'Value',ye);
set(handles.ye_position,'String',ye);

% --- Executes during object creation, after setting all properties.
function xe_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xe_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function xe_slider_Callback(hObject, eventdata, handles)
% hObject    handle to xe_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

xe = round(get(hObject,'Value')*10)/10;
set(handles.xe_slider,'Value',xe);
set(handles.xe_position,'String',xe);


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.xCenter,'String',0);
set(handles.xCenter_slider,'Value',0);
set(handles.yCenter,'String',0);
set(handles.yCenter_slider,'Value',0);
set(handles.x_radius,'String',5);
set(handles.x_radius_slider,'Value',5);
set(handles.y_radius,'String',5);
set(handles.y_radius_slider,'Value',5);
set(handles.ellipse_angle,'String',0);
set(handles.ellipse_angle_slider,'Value',0);
set(handles.weightQo,'String',1);
set(handles.Qo_slider,'Value',1);
set(handles.weightQp,'String',1);
set(handles.Qp_slider,'Value',1);
set(handles.simTime,'String',15);
set(handles.simTime_slider,'Value',15);
set(handles.revTime,'String',10);
set(handles.time_slider,'Value',10);

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of stop




