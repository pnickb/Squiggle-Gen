function varargout = ppbbtest(varargin)
% PPBBTEST MATLAB code for ppbbtest.fig
%      PPBBTEST, by itself, creates a new PPBBTEST or raises the existing
%      singleton*.
%
%      H = PPBBTEST returns the handle to a new PPBBTEST or the handle to
%      the existing singleton*.
%
%      PPBBTEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PPBBTEST.M with the given input arguments.
%
%      PPBBTEST('Property','Value',...) creates a new PPBBTEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ppbbtest_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ppbbtest_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ppbbtest

% Last Modified by GUIDE v2.5 02-Mar-2017 18:54:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ppbbtest_OpeningFcn, ...
                   'gui_OutputFcn',  @ppbbtest_OutputFcn, ...
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


% --- Executes just before ppbbtest is made visible.



function ppbbtest_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ppbbtest (see VARARGIN)

% Choose default command line output for ppbbtest
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ppbbtest wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ppbbtest_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function x1_Callback(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x1 as text
%        str2double(get(hObject,'String')) returns contents of x1 as a double
handles.x1 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function x1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y1_Callback(hObject, eventdata, handles)
% hObject    handle to y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y1 as text
%        str2double(get(hObject,'String')) returns contents of y1 as a double
handles.y1 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function y1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x2_Callback(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x2 as text
%        str2double(get(hObject,'String')) returns contents of x2 as a double
handles.x2 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function x2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y2_Callback(hObject, eventdata, handles)
% hObject    handle to y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y2 as text
%        str2double(get(hObject,'String')) returns contents of y2 as a double
handles.y2 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function y2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function b2_Callback(hObject, eventdata, handles)
% hObject    handle to b2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of b2 as text
%        str2double(get(hObject,'String')) returns contents of b2 as a double
handles.b2 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function b2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function b1_Callback(hObject, eventdata, handles)
% hObject    handle to b1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of b1 as text
%        str2double(get(hObject,'String')) returns contents of b1 as a double
handles.b1 = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function b1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
cla
axis equal
x = turn(handles.x1, handles.y1, handles.b1, handles.x2, handles.y2, handles.b2, 5);


function [distance] = turn(x1, y1, b1, x2, y2, b2, r)
 
b1 = mod(b1, 360);
b2 = mod(b2, 360);

db = mod(180,b2-b1);

distance = sqrt((x2-x1)^2+(y2-y1)^2);

%First, calculate the centers of the circles

%find the right and lefthand turn centers of point 1
    point1rhx = x1 + r*cosd(b1);
    point1rhy = y1 - r*sind(b1);
    point1lhx = x1 - r*cosd(b1);
    point1lhy = y1 + r*sind(b1);
%Find the righthand and lefthand turn centers of point 2
    point2rhx = x2 + r*cosd(b2);
    point2rhy = y2 - r*sind(b2);
    point2lhx = x2 - r*cosd(b2);
    point2lhy = y2 + r*sind(b2);
   
    

if(b1 == b2 && abs(atand((x2-x1)/(y2-y1))-b1)< .05)% check if both points are on the same line
    line([x1,x2],[y1,y2])  % If so, just draw a line between them
elseif abs(point2lhx - point1lhx)<.05*distance && abs(point2lhy - point1lhy)<.01*distance %% left hand circles have the same center %Check if both points are on the same circle
    a1 = atand((y1 - point1lhy)/(x1 - point1lhx));
    a2 = atand((y2 - point2lhy)/(x2 - point2lhx));
    if x1 == point1lhx && y1 > point1lhy
        a1 = 90;
    elseif x1 > point1lhx && y1 == point1lhy
        a1 = 0;
    elseif x1 > point1lhx && y1 < point1lhy
        a1 = 360 + a1;
    elseif x1 == point1lhx && y1 < point1lhy
        a1 = 270;
    elseif x1 < point1lhx && y1 < point1lhy
        a1 = 180 + a1;
    elseif x1 < point1lhx && y1 == point1lhy
        a1 = 180;
    elseif x1 < point1lhx && y1 > point1lhy
        a1 = 180+a1;
    end
    
    if x2 == point2lhx && y2 > point2lhy
        a2 = 90;
    elseif x2 > point2lhx && y2 == point2lhy
        a2 = 0;
    elseif x2 > point2lhx && y2 < point2lhy
        a2 = 360 + a2;
    elseif x2 == point2lhx && y2 < point2lhy
        a2 = 270;
    elseif x2 < point2lhx && y2 < point2lhy
        a2 = 180 + a2;
    elseif x2 < point2lhx && y2 == point2lhy
        a2 = 180;
    elseif x2 < point2lhx && y2 > point2lhy
        a2 = 180 + a2;
    end    
    
    if a1>a2
        a1 = a1-360;
    end
    DrawArc(point1lhx, point1lhy, a1, a2, r)
    distance = (a2-a1)*r*pi/180;
    
elseif abs(point2rhx - point1rhx)<.01*distance && abs(point2rhy - point1rhy)<.01*distance %% both right hand circles have the same center
    a1 = atand((y1 - point1rhy)/(x1 - point1rhx));
    a2 = atand((y2 - point2rhy)/(x2 - point2rhx));
    
    if x1 == point1rhx && y1 > point1rhy
        a1 = 90;
    elseif x1 > point1rhx && y1 == point1rhy
        a1 = 0;
    elseif x1 > point1rhx && y1 < point1rhy
        a1 = 360 + a1;
    elseif x1 == point1rhx && y1 < point1rhy
        a1 = 270;
    elseif x1 < point1rhx && y1 < point1rhy
        a1 = 180 + a1;
    elseif x1 < point1rhx && y1 == point1rhy
        a1 = 180;
    elseif x1 < point1rhx && y1 > point1rhy
        a1 = 180+a1;
    end
    
    if x2 == point2rhx && y2 > point2rhy
        a2 = 90;
    elseif x2 > point2rhx && y2 == point2rhy
        a2 = 0;
    elseif x2 > point2rhx && y2 < point2rhy
        a2 = 360 + a2;
    elseif x2 == point2rhx && y2 < point2rhy
        a2 = 270;
    elseif x2 < point2rhx && y2 < point2rhy
        a2 = 180 + a2;
    elseif x2 < point2rhx && y2 == point2rhy
        a2 = 180;
    elseif x2 < point2rhx && y2 > point2rhy
        a2 = 180 + a2;
    end    
    
    if a2>a1
        a2 = a2-360;
    end
    distance = (a1-a2)*r*pi/180;
    DrawArc(point1rhx, point1rhy, a1, a2, r)
  % END OF TURN LOGIC BASED ON BOTH POINTS BEING ON THE SAME CIRCLE
  
elseif distance >( r * 2)
    %convert bearings to angles from the +x axis
    b1 = (90-b1);
    b2 = (90-b2);
    
    dx = point2lhx - point1rhx;
    dy =  point2lhy - point1rhy;
    d = sqrt(dx^2+dy^2);
    L = sqrt(d^2-(2*r)^2);
    syms x
    %y = double(solve(y^2*(-dx-L) + 4*r*y + L - dx == 0, y))
    %alpha = 2*atan(y(2))*180/pi
    y = real(double(solve(L*cos(x)+ 2*r*sin(x)-dx == 0))*180/pi);
    alpha = y(1);

    Lx = L*cosd(alpha);
    Ly = L*sind(alpha);
    
    linex1 = point1rhx+ r*sind(alpha);
    linex2 = point2lhx- r*sind(alpha);
    liney1 = point1rhy+ r*cosd(alpha);
    liney2 = point2lhy- r*cosd(alpha);
    lineangle = atand((liney2-liney1)/(linex2-linex1))
    
    anglel2toc2 = (atand((point2lhy-liney2)/(point2lhx-linex2)) + 90-lineangle) % angle from point 2 of the line connecting the two arcs toward the center of point 2's lefthand circle.
    
    if  abs(anglel2toc2)>.01 && abs(anglel2toc2-180)> .01
        altalpha = 1
        alpha = y(2);
        linex1 = point1rhx+ r*sind(alpha);
        linex2 = point2lhx- r*sind(alpha);
        liney1 = point1rhy+ r*cosd(alpha);
        liney2 = point2lhy- r*cosd(alpha);
        lineangle = atand((liney2-liney1)/(linex2-linex1));
    end
    
    
    theta = b1+alpha; %Angle of travel on point 1's circle
    phi = b2+alpha;
    arc1a1 =b1+90-theta;
    arc1a2 =b1+90;
    if arc1a1 > arc1a2
        %changed1=1
        arc1a1 = arc1a1 - 360;
    end
    arc2a1 =b2-90-phi;
    arc2a2 =b2-90;
    if arc2a1 > arc2a2
        %changed2=1
        arc2a1 = arc2a1 - 360;
    end
    DrawArc(point1rhx, point1rhy, arc1a1, arc1a2, r)
     line([linex1,linex2], [liney1,liney2])
     DrawArc(point2lhx, point2lhy, arc2a1, arc2a2, r)
    DrawArc(x1, y1, 0, 360,.5)
    DrawArc(x2, y2, 0, 360,.5)
end

function DrawArc (x, y, a1, a2, r)
theta = linspace(a1, a2, 100);
xcords = x + r.*cosd(theta);
ycords = y + r.*sind(theta);
for i = 2:100
    line([xcords(i-1), xcords(i)], [ycords(i-1), ycords(i)]);
end
