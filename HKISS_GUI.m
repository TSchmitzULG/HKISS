function varargout = HKISS_GUI(varargin)
% HKISS_GUI MATLAB code for HKISS_GUI.fig
%      HKISS_GUI, by itself, creates a new HKISS_GUI or raises the existing
%      singleton*.
%
%      H = HKISS_GUI returns the handle to a new HKISS_GUI or the handle to
%      the existing singleton*.
%
%      HKISS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HKISS_GUI.M with the given input arguments.
%
%      HKISS_GUI('Property','Value',...) creates a new HKISS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before HKISS_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to HKISS_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help HKISS_GUI

% Last Modified by GUIDE v2.5 26-Apr-2017 11:36:19
warning('off','MATLAB:uitabgroup:OldVersion');
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @HKISS_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @HKISS_GUI_OutputFcn, ...
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


% --- Executes just before HKISS_GUI is made visible.
function HKISS_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to HKISS_GUI (see VARARGIN)

%Panel
handles.tgroup = uitabgroup('Parent', handles.figure1,'TabLocation', 'top');
handles.tab1 = uitab('Parent', handles.tgroup, 'Title', 'Sweep generation');
handles.tab2 = uitab('Parent', handles.tgroup, 'Title', 'Hammerstein kernels calculation');
handles.tab3 = uitab('Parent', handles.tgroup, 'Title', 'Nonlinear convolution');
handles.tab4 = uitab('Parent', handles.tgroup, 'Title', 'Help');

%Place panels into each tab
set(handles.P1,'Parent',handles.tab1);
set(handles.P2,'Parent',handles.tab2);
set(handles.P3,'Parent',handles.tab3);
set(handles.P4,'Parent',handles.tab4);

%Reposition each panel to same location as panel 1
set(handles.P2,'position',get(handles.P1,'position'));
set(handles.P3,'position',get(handles.P1,'position'));
set(handles.P4,'position',get(handles.P1,'position'));

%Default Param Model
N = 882996;
handles.N = N;
f1 = 5;
f2 = 22000;
handles.f1 = f1;
handles.f2 = f2;
lossL = 0.3 ;
lossH = 0.01 ;
handles.lossL = lossL;
handles.lossH = lossH;
fs = 44100;
handles.fs = fs;
fCutL = f1*(1+lossL);
handles.fCutL = fCutL;
fCutH = f2*(1-lossH);
handles.fCutH = fCutH;
nbKernels = 6;
handles.nbKernels = nbKernels;
g = zeros(1,1);
handles.g = g;
G = zeros(1,1);
handles.G = G;
handles.x = 0;
handles.y = 0;
hv = zeros(1,1);
handles.hv = hv;
maxKernelsSize = 2^15;
handles.maxKernelsSize = maxKernelsSize;
decayCut = 0;
handles.decayCut = decayCut;
handles.viewFFT_h_checkbox_value = 0;
handles.viewFFT_g_checkbox_value = 0;
handles.nLConvolutionResult = 0;
% Log Sweep creation
[sweep,R] = logSweep(N,f1,f2,[handles.lossL handles.lossH],fs);
handles.sweep = sweep;
handles.R = R;
[InvSweep] = invSweepAn(N,R,f1,fs);
handles.InvSweep = InvSweep;
invSweep = ifft(InvSweep,'symmetric');
handles.invSweep = invSweep;

[deltaSamples,trueDeltaSamples] = nbSamplesBetweenHarmo(nbKernels,R);
handles.deltaSamples = deltaSamples;
handles.trueDeltaSamples = trueDeltaSamples;
volterraKernelsSize = maxVolterraKernelsSize(N,maxKernelsSize,nbKernels,deltaSamples,decayCut);
handles.volterraKernelsSize=volterraKernelsSize;
handles.deconvSwInvSw = convFreq(sweep,invSweep);
plot(handles.axes1,handles.deconvSwInvSw),title(handles.axes1,'Deconvolving ESS (time View) ');
set(handles.startFrequency1_edit,'string',num2str(f1,4));
set(handles.stopFrequency1_edit,'string',num2str(f2,5));
set(handles.sweepLength1_edit,'string',num2str(N,6));
set(handles.sampleFrequency_edit,'string',num2str(fs,5));
set(handles.lossL_edit,'string',num2str(lossL,4));
set(handles.lossH_edit,'string',num2str(lossH,4));
set(handles.fCutL_edit,'string',num2str(fCutL,4));
set(handles.fCutH_edit,'string',num2str(fCutH,6));
set(handles.nbKernels_edit,'string',num2str(nbKernels,2));
set(handles.maxKernelsSize_edit,'string',num2str(maxKernelsSize,6));
set(handles.decayCut_edit,'string',num2str(handles.decayCut,4));
set(handles.viewKernel_g_edit,'string',num2str(1,2));
set(handles.viewKernel_h_edit,'string',num2str(1,2));
% Choose default command line output for HKISS_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes HKISS_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = HKISS_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;






function startFrequency1_edit_Callback(hObject, eventdata, handles)
% hObject    handle to startFrequency1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of startFrequency1_edit as text
%        str2double(get(hObject,'String')) returns contents of startFrequency1_edit as a double
         
   

% --- Executes during object creation, after setting all properties.
function startFrequency1_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to startFrequency1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stopFrequency1_edit_Callback(hObject, eventdata, handles)
% hObject    handle to stopFrequency1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stopFrequency1_edit as text
%        str2double(get(hObject,'String')) returns contents of stopFrequency1_edit as a double


% --- Executes during object creation, after setting all properties.
function stopFrequency1_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stopFrequency1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sweepLength1_edit_Callback(hObject, eventdata, handles)
% hObject    handle to sweepLength1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sweepLength1_edit as text
%        str2double(get(hObject,'String')) returns contents of sweepLength1_edit as a double


% --- Executes during object creation, after setting all properties.
function sweepLength1_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sweepLength1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sampleFrequency_edit_Callback(hObject, eventdata, handles)
% hObject    handle to sampleFrequency_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sampleFrequency_edit as text
%        str2double(get(hObject,'String')) returns contents of sampleFrequency_edit as a double


% --- Executes during object creation, after setting all properties.
function sampleFrequency_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sampleFrequency_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in generate_pushbutton.
function generate_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to generate_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    f1 = str2num(get(handles.startFrequency1_edit,'string'));
    handles.f1 = f1;
    f2 = str2num(get(handles.stopFrequency1_edit,'string'));
    handles.f2 = f2; 
    handles.N = str2num(get(handles.sweepLength1_edit,'string'));
     
    fs = str2num(get(handles.sampleFrequency_edit,'string'));
    handles.fs = fs; 
    lossL = str2num(get(handles.lossL_edit,'string'));
    lossH = str2num(get(handles.lossH_edit,'string'));
    handles.lossL = lossL; 
    handles.lossH = lossH;
    [handles.sweep,handles.R] = logSweep(handles.N,f1,f2,[lossL lossH],fs);
    [handles.InvSweep] = invSweepAn(handles.N,handles.R,handles.f1,handles.fs);
    invSweep = ifft(handles.InvSweep,'symmetric');
    handles.invSweep = invSweep;
    handles.deconvSwInvSw = convFreq(handles.sweep,invSweep);
   
val = get(handles.plot_type,'Value');
str = get(handles.plot_type, 'String');
switch str{val}
    case 'Time Domain'
        plot(handles.axes1,handles.deconvSwInvSw),title(handles.axes1,'Deconvolving ESS (time view)');
    case 'Frequency Domain'
        viewFft(handles.deconvSwInvSw,[handles.f1 handles.f2],[20 120], handles.fs,'-',handles.axes1),title(handles.axes1,'Deconvolving ESS (Frequency view)');
end;
if exist('audiowrite')
audiowrite('sweep.wav',handles.sweep,fs,'BitsPerSample',32);
else
    wavwrite(handles.sweep,fs,'sweep');
end;
guidata(hObject, handles);

% --- Executes on selection change in plot_type.
function plot_type_Callback(hObject, eventdata, handles)
% hObject    handle to plot_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns plot_type contents as cell array
%        contents{get(hObject,'Value')} returns selected item from plot_type
val = get(hObject,'Value');
str = get(hObject, 'String');
switch str{val}
    case 'Time Domain'
        plot(handles.axes1,handles.deconvSwInvSw),title(handles.axes1,'Deconvolving ESS (time view)');
    case 'Frequency Domain'
        viewFft(handles.deconvSwInvSw,[handles.f1 handles.f2],[20 120], handles.fs,'-',handles.axes1),title(handles.axes1,'Deconvolving ESS (Frequency view)');
end;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function plot_type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plot_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in LoadyResponse_pushbutton.
function LoadyResponse_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to LoadyResponse_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName_y,filePath_y]=uigetfile({'*.wav','Wav Files'},'Select output signal from DUT');
if exist('audioread')
    y = audioread([filePath_y fileName_y]);
else
    y = wavread([filePath_y fileName_y]);
end;
handles.y = y;
plot(handles.axes2,y),title(handles.axes2,'Output signal from our Nonlinear system under test y[n]');
guidata(hObject, handles);


% --- Executes on button press in Deconvolve_pushbutton.
function Deconvolve_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Deconvolve_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
len = 2^ceil(log2(length(handles.y)));
handles.len = len;
%invSweep
[handles.InvSweep] = invSweepAn(len,handles.R,handles.f1,handles.fs);

%Deconvolution
Y = fft(handles.y,len);
Z=Y.*handles.InvSweep;
Z(1)=0;
z = ifft(Z,'symmetric');
z = [z;z(1:round(length(z)/2)+handles.volterraKernelsSize(1))];
z = z./max(abs(z));
handles.z = z;
plot(handles.axes2,z),title(handles.axes2,'z[n] : Deconvolution of y[n] by the inverse ESS');
guidata(hObject, handles);

% --- Executes on key press with focus on Deconvolve_pushbutton and none of its controls.
function Deconvolve_pushbutton_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to Deconvolve_pushbutton (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function nbKernels_edit_Callback(hObject, eventdata, handles)
% hObject    handle to nbKernels_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nbKernels_edit as text
%        str2double(get(hObject,'String')) returns contents of nbKernels_edit as a double
    nbKernels = str2num(get(handles.nbKernels_edit,'string'));
    handles.nbKernels = nbKernels;
    [deltaSamples,trueDeltaSamples] = nbSamplesBetweenHarmo(nbKernels,handles.R);
    handles.deltaSamples = deltaSamples;
    handles.trueDeltaSamples = trueDeltaSamples;
    handles.volterraKernelsSize = maxVolterraKernelsSize(handles.N,handles.maxKernelsSize,handles.nbKernels,handles.deltaSamples);
    guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function nbKernels_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nbKernels_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxKernelsSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to maxKernelsSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxKernelsSize_edit as text
%        str2double(get(hObject,'String')) returns contents of maxKernelsSize_edit as a double
    maxKernelsSize = str2num(get(handles.maxKernelsSize_edit,'string'));
    handles.maxKernelsSize = maxKernelsSize;
    guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function maxKernelsSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxKernelsSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Cut_pushbutton.
function Cut_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Cut_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[deltaSamples,trueDeltaSamples] = nbSamplesBetweenHarmo(handles.nbKernels,handles.R);
volterraKernelsSize = maxVolterraKernelsSize(handles.N,handles.maxKernelsSize,handles.nbKernels,deltaSamples,handles.decayCut);
handles.volterraKernelsSize = volterraKernelsSize;
g= extractionKernel(handles.z,handles.len+1,deltaSamples,trueDeltaSamples,volterraKernelsSize,handles.nbKernels,-1000,0);
G = zeros(length(g),handles.nbKernels);
for i=1:handles.nbKernels
    G(:,i)=fft(g(:,i));
end;
handles.g= g;
handles.G = G;
plot(handles.axes2,g(:,1)),title(handles.axes2,'g_1[n] : First cut');
guidata(hObject, handles);

function decayCut_edit_Callback(hObject, eventdata, handles)
% hObject    handle to decayCut_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of decayCut_edit as text
%        str2double(get(hObject,'String')) returns contents of decayCut_edit as a double
handles.decayCut = str2num(get(handles.decayCut_edit,'string'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function decayCut_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to decayCut_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in HammersteinCompute_pushbutton.
function HammersteinCompute_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to HammersteinCompute_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 Hv = computeKernel(handles.G,1,handles.f1,handles.R,handles.fs);
for i = 1:handles.nbKernels
    hvTemp(:,i) = real(ifft(Hv(:,i),'symmetric'));
    hv(:,i)= [hvTemp(handles.decayCut+1000+1:end,i) ; zeros(handles.decayCut+1000,1)];
end;
handles.hv = hv;
viewFft(hv(:,1),round([handles.f1 handles.f2]),[-40 20], handles.fs,'-',handles.axes2);title(handles.axes2,'h_1[n] (Frequency view)');
assignin('base','HammersteinKernels',hv);
guidata(hObject, handles);



% --- Executes on button press in inputSignal_pushbutton.
function inputSignal_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to inputSignal_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fileName_x,filePath_x]=uigetfile({'*.wav','Wav Files'},'Select output signal from DUT');
if exist('audioread')
    x = audioread([filePath_x fileName_x]);
else
    x = wavread([filePath_x fileName_x]);
end;
handles.x = x;
plot(handles.axes3,x),title(handles.axes3,'Input signal x[n]');
guidata(hObject, handles);

% --- Executes on button press in convolve_pushbutton.
function convolve_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to convolve_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xM = signalPower(handles.x,handles.nbKernels,200);
assignin('base','xM',xM);
nLConvolutionResult = nLConvolution(xM,handles.hv);
plot(handles.axes3,nLConvolutionResult),title(handles.axes3,'NL convolution between x[n] and the Hammerstein kernels');
handles.nLConvolutionResult = nLConvolutionResult;
guidata(hObject, handles);



function viewKernel_g_edit_Callback(hObject, eventdata, handles)
% hObject    handle to viewKernel_g_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of viewKernel_g_edit as text
%        str2double(get(hObject,'String')) returns contents of viewKernel_g_edit as a double
    val = str2num(get(handles.viewKernel_g_edit,'string'));
    if handles.viewFFT_g_checkbox_value
    viewFft(handles.g(:,val),[handles.f1 handles.f2],[-40 20], handles.fs,'-',handles.axes2);title(handles.axes2,['g' num2str(val) '[n] (Frequency view)']);
    else
    plot(handles.axes2,handles.g(:,val)),title(handles.axes2,['g' num2str(val) '[n]']);
    end;

% --- Executes during object creation, after setting all properties.
function viewKernel_g_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to viewKernel_g_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function viewKernel_h_edit_Callback(hObject, eventdata, handles)
% hObject    handle to viewKernel_h_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of viewKernel_h_edit as text
%        str2double(get(hObject,'String')) returns contents of viewKernel_h_edit as a double
    val = str2num(get(handles.viewKernel_h_edit,'string'));
    if handles.viewFFT_h_checkbox_value
    viewFft(handles.hv(:,val),round([handles.f1 handles.f2]),[-40 20], handles.fs,'-',handles.axes2);title(handles.axes2,['h' num2str(val) '[n] (Frequency view)']);
    else
    plot(handles.axes2,handles.hv(:,val)),title(handles.axes2,['h' num2str(val) '[n]']);
    end;

% --- Executes during object creation, after setting all properties.
function viewKernel_h_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to viewKernel_h_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in viewFFT_g_checkbox.
function viewFFT_g_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to viewFFT_g_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of viewFFT_g_checkbox
handles.viewFFT_g_checkbox_value = get(hObject,'Value');
val = str2num(get(handles.viewKernel_g_edit,'string'));
    if handles.viewFFT_g_checkbox_value
    viewFft(handles.g(:,val),round([handles.f1 handles.f2]),[-40 20], handles.fs,'-',handles.axes2);title(handles.axes2,['g' num2str(val) '[n] (Frequency view)']);
    else
    plot(handles.axes2,handles.g(:,val)),title(handles.axes2,['g' num2str(val) '[n]']);
    end;
guidata(hObject, handles);


% --- Executes on button press in viewFFT_h_checkbox.
function viewFFT_h_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to viewFFT_h_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of viewFFT_h_checkbox
handles.viewFFT_h_checkbox_value = get(hObject,'Value');
val = str2num(get(handles.viewKernel_h_edit,'string'));
    if handles.viewFFT_h_checkbox_value
    viewFft(handles.hv(:,val),round([handles.f1 handles.f2]),[-40 20], handles.fs,'-',handles.axes2);title(handles.axes2,['h' num2str(val) '[n] (Frequency view)']);
    else
    plot(handles.axes2,handles.hv(:,val)),title(handles.axes2,['h' num2str(val) '[n]']);
    end;
guidata(hObject, handles);


% --- Executes on button press in exportParameters_pushbutton.
function exportParameters_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to exportParameters_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
assignin('base','f1',handles.f1);
assignin('base','f2',handles.f2);
assignin('base','N',handles.N);
assignin('base','fs',handles.fs);
assignin('base','lossL',handles.lossL);
assignin('base','lossH',handles.lossH);
assignin('base','sweep',handles.sweep);
assignin('base','invSweep',handles.invSweep);
assignin('base','R',handles.R);


% --- Executes on button press in exportResult_pushbutton.
function exportResult_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to exportResult_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
assignin('base','emulatedSignal',handles.nLConvolutionResult);





% --- Executes on button press in reset_pushbutton.
function reset_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to reset_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
evalin('base','clear all')



function lossL_edit_Callback(hObject, eventdata, handles)
% hObject    handle to lossL_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    lossL = str2num(get(handles.lossL_edit,'string'));
    handles.lossL = lossL;
    fCutL = handles.f1*(1+lossL);
    set(handles.fCutL_edit,'string',num2str(fCutL,6));
    guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function lossL_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lossL_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lossH_edit_Callback(hObject, eventdata, handles)
% hObject    handle to lossH_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    lossH = str2num(get(handles.lossH_edit,'string'));
    handles.lossH = lossH;
    fCutH = handles.f2*(1-lossH);
    set(handles.fCutH_edit,'string',num2str(fCutH,6));
    guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function lossH_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lossH_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
