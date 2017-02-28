function varargout = planner004(varargin)
% PLANNER004 MATLAB code for planner004.fig
%
%     PLANNER004 Is a flight path planner designed to help plan missions
%
%      for CReSIS. When run, planner004 will display a GUI and allow user
%
%      input of mission details. It will output a graph of the mission
%
%      flight plan, a text file of GPS waypoints, and some relevant warnings
%
%      and suggestions regarding mission planning.
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help planner004

% Last Modified by GUIDE v2.5 15-Feb-2017 16:19:28

% Begin initialization code - DO NOT EDIT00
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @planner004_OpeningFcn, ...
                   'gui_OutputFcn',  @planner004_OutputFcn, ...
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


% --- Executes just before planner004 is made visible.
function planner004_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = planner004_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in PlotButton.
function PlotButton_Callback(hObject, eventdata, handles)

%Alrighty, the variables you have so far are: (handles. in front of
%everything)


%SciTarLat, SciTarLong, CustTLat, CustTLong, NumLines, LineHeading,
%LineLength, LineSpace, CustRange, Cruise, SpeedConversion,
%RangeConversion, CustAPLat, CustAPLong, APLat, APLong


%If a radio button hasn't been selected in every field, the next line makes
%sure the plot button does nothing
if (isfield(handles,'SciTarLat') == 1) && (isfield(handles,'SciTarLong') == 1) && (isfield(handles,'NumLines') == 1) && (isfield(handles,'LineHeading') == 1) && (isfield(handles,'LineLength') == 1) && (isfield(handles,'LineSpace') == 1) && (isfield(handles,'SpeedConversion') == 1)
    %This function checks for custom inputs, and assigns appropriate values
    %to variables
    handles = CustomInput(hObject, handles);
    
    TotalDistance = handles.NumLines*handles.LineLength;%Adds the length flown in the flight lines to the total distance
    
    %The next line checks that the CustomInput function completed
    %successfully.
    if isfield(handles, 'fail') == 0
        
        %creates matrices containing latitudes and longitudes of waypoints
        [handles.lats, handles.longs] = GenFlightLines(handles);
        
        %Checks for a custom bank angle, otherwise sets to 15 degrees
        if isfield(handles,'CustBankAngle') == 0;
            handles.CustBankAngle = 15;
        end
  
        %Converts the waypoint coordinates from lat, long to cartesian.
        [handles.XCords, handles.YCords] = MercConv(handles.lats, handles.longs);
        
       %Scales the x and y coordinates to the user's chosen units in order
       %to make the graph legible
        [handles.XCords, handles.YCords] = Scale(handles.XCords, handles.YCords,handles.lats,handles.longs, handles);
        handles.TurnRadius = handles.Cruise*2/(9.8*tand(handles.CustBankAngle)) / handles.RangeConversion %Turn Radius is used by the turn function in a way that requires it to be in the users units
        
        
        cla %clears previously plotted flight paths
        
        for i = 1:2:2*handles.NumLines
            line([handles.XCords(i) handles.XCords(i+1)],[handles.YCords(i) handles.YCords(i+1)])
        end
        if handles.TurnRadius<handles.LineSpace*3
            for i = 2:2:2*handles.NumLines-2
                TotalDistance = TotalDistance + turn(handles.XCords(i),handles.YCords(i),handles.LineHeading+mod(1+mod(i/2,2),2)*180,handles.XCords(i+1), handles.YCords(i+1),handles.LineHeading + 180*mod(i/2,2), handles.TurnRadius);
            end
        else
            %Do those big turns
        end
        
        axis equal
        %%The next if statement checks if the user entered a file name. If
        %%so, it outputs the coordinates to a text file.
        if isfield(handles,'CustFileName') == 1
           handles.Coords = vertcat(handles.lats, handles.longs);
           %If there are problems with reading from the output file using
           %matlab, try making the second argument in the next line 'w'
           %instead of 'wt'
           handles.fileID = fopen([char(handles.CustFileName) '.txt'],'wt');
           
           fprintf(handles.fileID,'%8.8f %8.8f \n',handles.Coords);
           fclose(handles.fileID);
           
           %Coords = horzcat(lats',longs')
           %dlmwrite('latslongs.txt',Coords,'precision',6)
        end
      
        
        %FINISHED GENERATING PATH - OUTPUT: Total distance, flight time
        TotalDistance = TotalDistance;
        time = TotalDistance * handles.RangeConversion/handles.Cruise/3600;
    end
    
else
    disp('You need to make a selection for every field')
    
end



function [distance] = turn(x1, y1, b1, x2, y2, b2, r)
 
b1 = mod(b1, 360);
b2 = mod(b2, 360);

db = mod(180,b2-b1);

distance = sqrt((x2-x1)^2+(y2-y1)^2);

%First, calculate the centers of the circles

%find the right and lefthand turn centers of point 1
    r1x = x1 + r*cosd(b1);
    r1y = y1 - r*sind(b1);
    l1x = x1 - r*cosd(b1);
    l1y = y1 + r*sind(b1);
%Find the righthand and lefthand turn centers of point 2
    r2x = x2 + r*cosd(b2);
    r2y = y2 - r*sind(b2);
    l2x = x2 - r*cosd(b2);
    l2y = y2 + r*sind(b2);
%Check if both points are on the same circle
if abs(l2x - l1x)<.05*distance && abs(l2y - l1y)<.01*distance %% left hand circles have the same center
    a1 = atand((y1 - l1y)/(x1 - l1x));
    a2 = atand((y2 - l2y)/(x2 - l2x));
    if x1 == l1x && y1 > l1y
        a1 = 90;
    elseif x1 > l1x && y1 == l1y
        a1 = 0;
    elseif x1 > l1x && y1 < l1y
        a1 = 360 + a1;
    elseif x1 == l1x && y1 < l1y
        a1 = 270;
    elseif x1 < l1x && y1 < l1y
        a1 = 180 + a1;
    elseif x1 < l1x && y1 == l1y
        a1 = 180;
    elseif x1 < l1x && y1 > l1y
        a1 = 180+a1;
    end
    
    if x2 == l2x && y2 > l2y
        a2 = 90;
    elseif x2 > l2x && y2 == l2y
        a2 = 0;
    elseif x2 > l2x && y2 < l2y
        a2 = 360 + a2;
    elseif x2 == l2x && y2 < l2y
        a2 = 270;
    elseif x2 < l2x && y2 < l2y
        a2 = 180 + a2;
    elseif x2 < l2x && y2 == l2y
        a2 = 180;
    elseif x2 < l2x && y2 > l2y
        a2 = 180 + a2;
    end    
    
    if a1>a2
        a1 = a1-360;
    end
    DrawArc(l1x, l1y, a1, a2, r)
    distance = (a2-a1)*r*pi/180;
    
    
elseif abs(r2x - r1x)<.01*distance && abs(r2y - r1y)<.01*distance %% both right hand circles have the same center
    a1 = atand((y1 - r1y)/(x1 - r1x));
    a2 = atand((y2 - r2y)/(x2 - r2x));
    
    if x1 == r1x && y1 > r1y
        a1 = 90;
    elseif x1 > r1x && y1 == r1y
        a1 = 0;
    elseif x1 > r1x && y1 < r1y
        a1 = 360 + a1;
    elseif x1 == r1x && y1 < r1y
        a1 = 270;
    elseif x1 < r1x && y1 < r1y
        a1 = 180 + a1;
    elseif x1 < r1x && y1 == r1y
        a1 = 180;
    elseif x1 < r1x && y1 > r1y
        a1 = 180+a1;
    end
    
    if x2 == r2x && y2 > r2y
        a2 = 90;
    elseif x2 > r2x && y2 == r2y
        a2 = 0;
    elseif x2 > r2x && y2 < r2y
        a2 = 360 + a2;
    elseif x2 == r2x && y2 < r2y
        a2 = 270;
    elseif x2 < r2x && y2 < r2y
        a2 = 180 + a2;
    elseif x2 < r2x && y2 == r2y
        a2 = 180;
    elseif x2 < r2x && y2 > r2y
        a2 = 180 + a2;
    end    
    
    if a2>a1
        a2 = a2-360;
    end
    distance = (a1-a2)*r*pi/180;
    DrawArc(r1x, r1y, a1, a2, r)
  % END OF TURN LOGIC BASED ON BOTH POINTS BEING ON THE SAME CIRCLE
elseif distance > r * 2
    %convert bearings to angles from the +x axis
    b1 = (90-b1);
    b2 = (90-b2);
    
    
end


function DrawArc (x, y, a1, a2, r)
theta = linspace(a1, a2, 100);
xcords = x + r.*cosd(theta);
ycords = y + r.*sind(theta);
for i = 2:100
    line([xcords(i-1), xcords(i)], [ycords(i-1), ycords(i)]);
end

function[x,y] = Scale(xcords,ycords,lats, longs,handles)
PlotDistance = sqrt((xcords(2)-xcords(1))^2+(ycords(2)-ycords(1))^2);
RealDistance = FlightDistance(lats(1), longs(1), lats(2), longs(2));
ScaleFactor = PlotDistance/RealDistance;
x = xcords./ScaleFactor/handles.RangeConversion;
y = ycords./ScaleFactor/handles.RangeConversion;

function[x,y] = MercConv(lats,longs)
R = 6371000;
radlats = lats.*pi./180;
radlongs = longs.*pi./180;
x = R.*cos(radlats(1)).*(longs-min(longs)).*pi./180;
y= R.*cos(radlats(1)).*log(tan(pi/4+(lats.*pi./180)./2));
y = y-y(1);

function [lat, long] = DegConv(x, y) %have this function convert from mercator coordinates to xy coordinates.
lat = x
long = y

function[lats, longs] = GenFlightLines(handles)
%This function generates a two matrices consisting of the latitudes and
%longitudes of the flight waypoints. The waypoints are at the beginning and
%end of each flight line. Waypoint n is characterized by lats(n), longs(n).
Bearing = handles.LineHeading;     %The bearing of the flight lines
length = handles.LineLength*handles.RangeConversion;    %The length of the flight lines, converted to meters
R = 6371000; %Approximate radius of the earth, in meters
space = handles.LineSpace*handles.RangeConversion;

lats(1) = handles.SciTarLat; %assumes your first point is at the latitude and longitude inputted in the science target latitude and longitude fields
longs(1) = handles.SciTarLong;
[lats(2), longs(2)] = NewPoint(lats(1), longs(1), Bearing, length); %Creates the 2nd point

for i= 2:handles.NumLines %Flight waypoints are created by this loop
    if mod(i, 2) == 0
        [lats(2*i-1), longs(2*i-1)] = NewPoint(lats(2*i-2), longs(2*i-2), Bearing+90, space); %After odd numbered lines, turns 90 degrees to the right to create the first waypoint of the next line
        [lats(2*i), longs(2*i)] = NewPoint(lats(2*i-1), longs(2*i-1), Bearing+180, length); %Creates the even numbered lines, 
    else
        [lats(2*i-1), longs(2*i-1)] = NewPoint(lats(2*i-2), longs(2*i-2), Bearing+90, space);
        [lats(2*i), longs(2*i)] = NewPoint(lats(2*i-1), longs(2*i-1), Bearing, length);
    end
end

function [newlat, newlong] = NewPoint(lat, long, bearing, d) %Outputs the latitude and longitude of a point found by travelling from an initial point a distance d in meters pointing at the inputted bearing
R = 6371000; %radius of the earth in meters
newlat = asind(sind(lat)*cos(d/R)+cosd(lat)*sin(d/R)*cosd(bearing));
A = sind(bearing)*sin(d/R)*cosd(lat);
B = cos(d/R) - sind(lat)*sind(newlat);
newlong = long + atan2d(A,B);

function [d] = FlightDistance(Lat1, Long1, Lat2, Long2) %Calculates the distance in meters between two points on the earth - - Might not be used in final program

    d= 2*6371000*asin(sqrt(sind((Lat2-Lat1)/2)^2 + cosd(Lat1)*cosd(Lat2)*sind((Long2-Long1)/2)^2));
    
function [handles] = CustomInput(hObject, handles) %Detects custom inputs, then assigns variables
if (handles.SciTarLat == -100)
    if (isfield(handles,'CustTLat') == 1) && (isfield(handles,'CustTLong') == 1)
        handles.SciTarLat = handles.CustTLat;
        handles.SciTarLong = handles.CustTLong;
    else
        display('You need to fill out every custom field when you select a custom option')
        handles.fail = 1;
        disp('failed for scitar')
    end
end
if (handles.APLat == -100) 
    if (isfield(handles,'CustAPLat') == 1) && (isfield(handles,'CustAPLong') == 1)
        handles.APLat = handles.CustAPLat;
        handles.APLong = handles.CustAPLong;

    else
        display('You need to fill out every custom field when you select a custom option')
        handles.fail = 1;
        disp('failed for aplat')
    end
end
if (handles.Cruise == -100) 
    if (isfield(handles,'CustCruise') == 1) && (isfield(handles,'CustRange') == 1)
        handles.Cruise = handles.CustCruise*handles.SpeedConversion;
        handles.Range = handles.CustRange*handles.RangeConversion;
    else
        display('You need to fill out every custom field when you select a custom option')
        handles.fail = 1;
        disp('failed for range')
    end
end

guidata(hObject, handles);

function FlightLines_Callback(hObject, eventdata, handles)


handles.NumLines = str2double(get(hObject,'String'));
guidata(hObject, handles);

function FlightLines_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FlightLines (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function LineLength_Callback(hObject, eventdata, handles)


handles.LineLength = str2double(get(hObject,'String')); %gets line length input
guidata(hObject, handles);

function LineLength_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function LineSpacing_Callback(hObject, eventdata, handles)


handles.LineSpace = str2double(get(hObject,'String')); %Gets line space input
guidata(hObject, handles);

function LineSpacing_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function LineHeading_Callback(hObject, eventdata, handles)
% hObject    handle to LineHeading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LineHeading as text
%        str2double(get(hObject,'String')) returns contents of LineHeading as a double

% --- Executes during object creation, after setting all properties.


heading = str2double(get(hObject,'String'));%Gets heading input
handles.LineHeading = mod(heading, 360); %If the user puts some silly bearing above 360, this lowers it to below 360
guidata(hObject, handles);

function LineHeading_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LineHeading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CustTLat_Callback(hObject, eventdata, handles)
% hObject    handle to CustTLat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustTLat as text
%        str2double(get(hObject,'String')) returns contents of CustTLat as a double

handles.CustTLat = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustTLat_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CustTLong_Callback(hObject, eventdata, handles)
% hObject    handle to CustTLong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustTLong as text
%        str2double(get(hObject,'String')) returns contents of CustTLong as a double
handles.CustTLongEntered = 1;
handles.CustTLong = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustTLong_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CustRange_Callback(hObject, eventdata, handles)
% hObject    handle to CustRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustRange as text
%        str2double(get(hObject,'String')) returns contents of CustRange as a double
handles.CustRangeEntered = 1;
handles.CustRange = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustRange_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CustCruise_Callback(hObject, eventdata, handles)
% hObject    handle to CustCruise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustCruise as text
%        str2double(get(hObject,'String')) returns contents of CustCruise as a double
handles.CustCruise = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustCruise_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function bgPlanes_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in bgPlanes 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)


switch get(eventdata.NewValue,'Tag')
    case 'rbSierra'
        handles.Cruise = 30.8667; %m/s
        handles.Range = 1018600; %m
    case 'rbP3'
        handles.Cruise = 169.767; %m/s
        handles.Range = 5556000; %m
    case 'rbTwinOtter'
        handles.Cruise = 56.5889; %m/s
        handles.Range = 1574200; %m
    case 'rbCustomPlane'
        handles.Cruise = -100;
        handles.Range = -100;
end

guidata(hObject, handles);

function bgScienceTargets_SelectionChangeFcn(hObject, eventdata, handles) 

%SciTarLat
%SciTarLong

%NEED TO ADD ANOTHER VARIABLE BASED ON RUNWAY BEARING
switch get(eventdata.NewValue,'Tag')
    case 'rbChamb'
        handles.SciTarLat = 76.743607;%
        handles.SciTarLong = -68.615041;%
    case 'rbCampC'
        handles.SciTarLat = 77.166696;%
        handles.SciTarLong = -61.133369;%
    case 'rbJakob'
        handles.SciTarLat = 69.215840;%
        handles.SciTarLong = -49.798696;%
    case 'rbRusse'
        handles.SciTarLat = 67.101912;%
        handles.SciTarLong = -50.225496;%
    case 'rbColumbiaG'
        handles.SciTarLat = 61.170380;%
        handles.SciTarLong = -147.026099;%
    case 'rbNuukG'
        handles.SciTarLat = 65.212518;%
        handles.SciTarLong = -50.662002;%
    case 'rbOtherTarget'
        handles.SciTarLat = -100;%
        handles.SciTarLong = -100;%
end
guidata(hObject, handles);

function bgUnits_SelectionChangeFcn(hObject, eventdata, handles)

%Creates conversion factors to convert everything into meters, meters per
%second, etc in order to avoid mental jumprope and mistakes in doing all
%the math
switch get(eventdata.NewValue,'Tag')
    case 'rbMeters'
        handles.SpeedConversion = 1;
        handles.RangeConversion = 1;
    case 'rbMiles'
        handles.SpeedConversion = 0.44704;
        handles.RangeConversion = 1609.34;
    case 'rbKnots'
        handles.SpeedConversion = 0.514444;
        handles.RangeConversion = 1852;
    case 'rbKm'
        handles.SpeedConversion = 0.277778;
        handles.RangeConversion = 1000;
end
guidata(hObject, handles);

function bgAirports_SelectionChangeFcn(hObject, eventdata, handles)
%Sets the latitude and longitude of the mission airport
%APLat, APLong

switch get(eventdata.NewValue,'Tag')
    case 'rbThule'
        handles.APLat = 77.46666666666667;%
        handles.APLong = -69.23055555555555;%
    case 'rbIlulissat'
        handles.APLat = 69.21666666666667;%
        handles.APLong = -51.1;%
    case 'rbKang'
        handles.APLat = 67.00861111111111;%
        handles.APLong = -50.689166666666665;%
    case 'rbNuuk'
        handles.APLat = 64.17500000000001;%
        handles.APLong = 51.73888888888889;%
    case 'rbValdez'
        handles.APLat = 61.85;%
        handles.APLong = -146.34833333333333;%
    case 'rbBarrow'
        handles.APLat = 71.29055555555556;%
        handles.APLong = -156.78861111111112;%
    case 'rbCustAP'
        handles.APLat = -100;%
        handles.APLong = -100;%


end
guidata(hObject, handles);

function CustAPLong_Callback(hObject, eventdata, handles)
% hObject    handle to CustAPLong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustAPLong as text
%        str2double(get(hObject,'String')) returns contents of CustAPLong as a double

handles.CustAPLong = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustAPLong_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CustAPLat_Callback(hObject, eventdata, handles)
% hObject    handle to CustAPLat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CustAPLat as text
%        str2double(get(hObject,'String')) returns contents of CustAPLat as a double
handles.CustAPLat = str2double(get(hObject,'String'));
guidata(hObject, handles);

function CustAPLat_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FileName_Callback(hObject, eventdata, handles)
% hObject    handle to FileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FileName as text
%        str2double(get(hObject,'String')) returns contents of FileName as a double
    handles.CustFileName = get(hObject,'String');
    guidata(hObject, handles);
% --- Executes during object creation, after setting all properties.
function FileName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BankAngle_Callback(hObject, eventdata, handles)
% hObject    handle to BankAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BankAngle as text
%        str2double(get(hObject,'String')) returns contents of BankAngle as a double
handles.CustBankAngle = str2double(get(hObject,'String'));
guidata(hObject, handles);
% --- Executes during object creation, after setting all properties.
function BankAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BankAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
