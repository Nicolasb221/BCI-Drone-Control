gds_interface = gtecDeviceInterface; %asigna la intefaz gtec.
%configurar la conexion.
gds_interface.IPAddressHost = '127.0.0.1';
gds_interface.IPAddressLocal = '127.0.0.1';
gds_interface.HostPort = 50223;
gds_interface.LocalPort = 50224;
%obtener dispositivos conectados actualemnte.
connected_devices = gds_interface.GetConnectedDevices();
%crear la configuracion del dispositivo.
gusbamp_config = gUSBampDeviceConfiguration();
gusbamp_config.Name = connected_devices(1,1).Name;
%seleccionar la interfaz a utilizar, en este caso GUSBAMP.
gds_interface.DeviceConfigurations = gusbamp_config;
%canales disponibles.
available_channels = gds_interface.GetAvailableChannels();
%configuracion muestreo: tasa de muestreo y numero de escaneos.
gusbamp_config.SamplingRate = 256;
gusbamp_config.NumberOfScans = 8;

%test de adquision, aqui se le pide al dispositivo gusbamp que genere una
%señal para luego graficarla en tiempo real.

% gusbamp_siggen = gUSBampInternalSignalGenerator();
% gusbamp_siggen.Enabled = true;
% gusbamp_siggen.Frequency = 1; % frecuencia 1 son 256
% gusbamp_siggen.WaveShape = 3; %forma de onda 1- cuadrada, 2- diente de sierra 3- sinusoidal 4- lineal 5- noise
% gusbamp_siggen.Amplitude = 10;
% gusbamp_siggen.Offset = 0;
% gusbamp_config.InternalSignalGenerator = gusbamp_siggen;

%registrar todos los canales disponibles
for i=1:size(gusbamp_config.Channels,2)
    if (available_channels(1,i))
        gusbamp_config.Channels(1,i).Available = true;
        gusbamp_config.Channels(1,i).Acquire = true;
        % do not use filters
        gusbamp_config.Channels(1,i).BandpassFilterIndex = -1;
        gusbamp_config.Channels(1,i).NotchFilterIndex = -1;
        % do not use a bipolar channel
        gusbamp_config.Channels(1,i).BipolarChannel = 0;
    end
end
% asignar configuracion a la interfaz.
gds_interface.DeviceConfigurations = gusbamp_config;
% configura la interfaz con los parametros actuales.
gds_interface.SetConfiguration();
% inicia la adquisicion
gds_interface.StartDataAcquisition();

%ver si dejar 256 o gusbamp_config.SamplingRate
Fs = gusbamp_config.SamplingRate; %Frecuencia de muestreo
NSc = gusbamp_config.NumberOfScans; %Número de scans correspondiente
n_lim = Fs*30; %total de muestras que quiero adquirir
time_lim = n_lim/double(Fs); % calculo el tiempo limite maximo
senal = single(zeros(n_lim, 16));%creo la matriz donde almacenare los valores de la señal
senal_Tam = length(senal);
time_vector = (1:senal_Tam)/double(Fs); % calculo el tiempo en el que ocurrira cada muestra
% v_sig_off = [0 3000 6000 9000 12000 15000 18000 21000 24000 27000 30000 33000 36000 39000 42000 45000];

samples_acquired = 0;

orden_filt= 16;

bpFilt = designfilt('bandpassiir','FilterOrder',orden_filt, ...
         'HalfPowerFrequency1',1,'HalfPowerFrequency2',100, ...
         'DesignMethod','butter','SampleRate',Fs);
     

         
     
w0 = 100/(double(Fs)/2);
bw = w0/35; % probar anchos de banda
[b,a] = iirnotch(w0,bw); 
% notch_Filt = filter(b,a,senal);
samples_2 = 0;
cont_n = 1;
% mostrar toda la ventana de tiempo.
Canales = {'Ch1','Ch2','Ch3','Ch4','Ch5','Ch6','Ch7','Ch8','Ch9','Ch10','Ch11','Ch12','Ch13','Ch14','Ch15','Ch16'};

%duración ventana
ventana=256*3; %3 seg


matriz_valores_prueba = zeros (10,1);

contador = 1;
%dron
Estimulo=0;
Distancia_defecto=0.5;
disp ('Program started');
sim= remApi ('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19998,true,true,5000,5);
    
    
while (samples_acquired < n_lim) %adquisicion 256 = 1 segundo -- modificar n_lim para cambiar tiempo de ventanas
   
    
    [scans_received, data] = gds_interface.GetData(NSc); % data es de tamaño 8x16 
    senal((samples_acquired + 1):(samples_acquired+scans_received),:) = data;
    samples_acquired = samples_acquired + scans_received;
    
    
    
    z = mod(samples_acquired,ventana);
    if z == 0
        %Para eliminar efecto de borde
        sen_bor = padarray(senal,[orden_filt 0],'symmetric', 'both'); %para reflejar las muestras en la señal de entrada y al final
        senal_filt_bp = filtfilt(bpFilt, double(sen_bor)); %filtrar señal filtro pasabanda
        senal_tot = (senal_filt_bp(orden_filt+1 : end-(orden_filt), :));
        senal_filt = filtfilt(bpFilt, double(senal(1:samples_acquired,:)));

        %     notch_Filt = filter(b,a,senal); %filtrar señal filtro notch, se filtra dos veces
        
        std_off= mean(std(senal,0,2)); %desviación estándar de la señal
        offset= 8*std_off; %offset valores estándar de 6 u 8
        vis_off= (0:15)'*offset; %offset para visualización
        mat_off= repmat(vis_off,1,n_lim); %matriz offset
        sen_off = (mat_off(:,orden_filt-8 : end-(orden_filt+1))); %para reflejar las muestras en la señal de entrada de offset
        
        senalconoff = senal_tot + mat_off'; %señal de visualización
        
        % se llena la matriz señal con los datos de la matriz data obtenidos en cada caso.
        plot(time_vector(1:samples_acquired), senalconoff(1:samples_acquired,:),'-b');
        
        %linea
        line([time_vector(samples_acquired) time_vector(samples_acquired)],[mat_off(1)-offset mat_off(end)+offset],'Marker','>','MarkerEdgeColor',[0.8500 0.3250 0.0980],'MarkerFaceColor',[0.8500 0.3250 0.0980],'Color', [0.8500 0.3250 0.0980],'LineWidth', 3)
        
        %limites
        xlim([0 time_lim]);
        ylim([(mat_off(1)-offset),(mat_off(end)+offset)]);
        set(gca,'YTick', vis_off,'YTickLabel', Canales)
        xlabel('Tiempo (s)')
        ylabel('Canales')
        title('Señales generadas separadas con offset')
        
        %sobreescribir muestras
        hold on
        plot(time_vector(1:samples_2), senalconoff(1:samples_2,:),'-b');
        hold off
        
        columna_1 = senal_tot(:,1);
        
%         prom_dron = 0;
%         if contador == 1
%             prom_dron = 0;
%             matriz_valores_prueba(contador, 1) = prom_dron;
%             contador = contador+1;
%         else
%             prom_dron = mean(columna_1(samples_acquired-ventana:samples_acquired).^2 , 1);
%             matriz_valores_prueba(contador, 1) = prom_dron;
%             contador = contador+1;
%         end
% 
%         if prom_dron >= 2.5e+09 %% revisar
%             Estimulo = 2;
%             if (clientID>-1)
%                 disp('Connected to remote API server');
%                 %inicio del programa
%                 %tiempo=60;%duracion de la conexion Matlab-Coppelia
%                 [returnCode,Ptarget]=sim.simxGetObjectHandle(clientID,'Quadricopter_target',sim.simx_opmode_blocking);
%                 [returnCode,P_ini_target]=sim.simxGetObjectPosition(clientID,Ptarget,-1,sim.simx_opmode_blocking);
%                 
%                 
%                 
%                 switch Estimulo
%                     
%                     case 1
%                         %despega
%                         P_ini_target(3)=P_ini_target(3)+Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                     case 2
%                         %Adelante
%                         P_ini_target(2)=P_ini_target(2)+Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                     case 3
%                         %Atras
%                         P_ini_target(2)=P_ini_target(2)-Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                     case 4
%                         %Derecha
%                         P_ini_target(1)=P_ini_target(1)+Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                     case 5
%                         %Izquierda
%                         P_ini_target(3)=P_ini_target(3)+Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                     case 6
%                         %Aterrizar
%                         P_ini_target(3)=P_ini_target(3)-Distancia_defecto;
%                         dato_vel = P_ini_target;
%                         [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
%                         Estimulo=0;
%                 end
%                 
%                 %
%             else
%                 disp('Failed connecting to remote API server');
%                 sim.delete();
%             end
% 
%         end
         
    end
    
        
    %Refresh de plot
    c = eq(samples_acquired,n_lim);%puede necesitar double
    
    if c == 1
        samples_2 = samples_acquired;
        samples_acquired = 0;
        cont_n= cont_n+1;
    end
    
    
   
    
    %Número de ventanas
    if cont_n == 4
        samples_2 = samples_acquired;
        samples_acquired= n_lim +1;
    end
   
end

sim.delete(); % finaliza la comunicacion.
disp('Program ended');






% %exportar datos a excel
% xlswrite('signal_data.xlsx',senal,'Hoja1','B3:Q2562')

gds_interface.StopDataAcquisition(); %detener adquisicion
delete(gds_interface);
clear gds_interface;