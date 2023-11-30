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
n_lim = Fs*1000; %total de muestras que quiero adquirir 1440 -- 1446 con despegue y aterrizaje
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


%duración ventana
ventana=256*3; %3 seg

%Vector de eventos
vec_eventos = zeros(1,ventana,10);

%Vector de movimientos
vec_mov = [1,2,3,4,5,6,7,8,9,10];
contador = 1;

%Tags
tags_modelo_nivel1 = [0];
contador_t =1;

%dron 
Estimulo=0;
Distancia_defecto=0.95;
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
        if contador_t > contador
            n_lim = 0;

        elseif vec_mov(samples_acquired/ventana) == vec_mov(contador)
            
            if vec_mov(contador)==1
                Estimulo = 1;
            end
            
            % 1=adelante,2=derecha,3=atras,4=izquierda // ETIQUETAS
            % AJUSTAR MOVIMIENTOS
            
            %Movimiento 1 adelante
            if vec_mov(contador)==2
                if tags_modelo_nivel1(contador_t) == 1
                    Estimulo = 2;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 2 adelante
            if vec_mov(contador)==3
                if tags_modelo_nivel1(contador_t) == 1
                    Estimulo = 2;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 3 adelante
            if vec_mov(contador)==4
                if tags_modelo_nivel1(contador_t) == 1
                    Estimulo = 2;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];   
                end
                contador_t = contador_t+1;
            end
            %Movimiento 4 derecha
            if vec_mov(contador)==5
                if tags_modelo_nivel1(contador_t) == 2
                    Estimulo = 4;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 5 derecha
            if vec_mov(contador)==6
                if tags_modelo_nivel1(contador_t) == 2
                    Estimulo = 4;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];   
                end
                contador_t = contador_t+1;
            end
            %Movimiento 6 derecha
            if vec_mov(contador)==7
                if tags_modelo_nivel1(contador_t) == 2
                    Estimulo = 4;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 7 adelante
            if vec_mov(contador)==8
                if tags_modelo_nivel1(contador_t) == 1
                    Estimulo = 2;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 8 adelante
            if vec_mov(contador)==9
                if tags_modelo_nivel1(contador_t) == 1
                    Estimulo = 2;
                else
                    vec_mov=[vec_mov(1:contador-1) vec_mov(contador) vec_mov(contador:end)];
                end
                contador_t = contador_t+1;
            end
            %Movimiento 9 aterrizaje
            if vec_mov(contador)==10
                Estimulo = 6;
                contador_t = contador_t+10;
            end 
        end
        

        contador = contador+1;

        if (clientID>-1)
            disp('ejecute');
            %inicio del programa
            %tiempo=60;%duracion de la conexion Matlab-Coppelia
            [returnCode,Ptarget]=sim.simxGetObjectHandle(clientID,'Quadricopter_target',sim.simx_opmode_blocking);
            [returnCode,P_ini_target]=sim.simxGetObjectPosition(clientID,Ptarget,-1,sim.simx_opmode_blocking);
                
            switch Estimulo
                
                case 1
                    %despega
                    P_ini_target(3)=P_ini_target(3)+Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
                case 2
                    %Adelante
                    P_ini_target(2)=P_ini_target(2)+Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
                case 3
                    %Atras
                    P_ini_target(2)=P_ini_target(2)-Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
                case 4
                    %Derecha
                    P_ini_target(1)=P_ini_target(1)+Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
                case 5
                    %Izquierda
                    P_ini_target(1)=P_ini_target(1)-Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
                case 6
                    %Aterrizar
                    P_ini_target(3)=P_ini_target(3)-Distancia_defecto;
                    dato_vel = P_ini_target;
                    [returnCode]=sim.simxSetObjectPosition(clientID,Ptarget,-1,dato_vel,sim.simx_opmode_oneshot)
                    Estimulo=0;
            end
            
            %
        else
            disp('Failed connecting to remote API server');
            sim.delete();
        end
        
        
        
        senal_filt_bp = filtfilt(bpFilt, double(senal(1:samples_acquired,1:10))); %filtrar señal filtro pasabanda
        %Rellenar vector de eventos
        vec_eventos(:,:,:)= senal_filt_bp(samples_acquired-ventana+1:samples_acquired,1:10);
        
        
        %Extracción de características
        %Promedio
        prom = mean (vec_eventos,2);
        per_prom = permute(prom,[2 1 3]);
        prom = squeeze(per_prom)';
        
        %Varianza
        vari = var (vec_eventos,[],2);
        per_var = permute(vari,[2 1 3]);
        vari = squeeze(per_var)';
        
        %Mediana
        med = median(vec_eventos,2);
        per_med = permute(med,[2 1 3]);
        med = squeeze(per_med)';
        
        %Moda
        moda = mode(vec_eventos,2);
        per_mod = permute(moda,[2 1 3]);
        moda = squeeze(per_mod)';
        
        
        %Densidad espectral de potencia
        psd_1 = squeeze(psd_1);
        [psd_2,freq] = pwelch(psd_1, 96, 48, 512, 256);
        vec_psd = mean(psd_2);
        
        % rangos frecuancias eeg
        [~,minloc_05] = min (abs(freq(1:250,:)-0.5));
        
        [~,minloc_4] = min (abs(freq(1:250,:)-4));
        
        [~,minloc_8]  = min (abs(freq(1:250,:)-8));
        
        [~,minloc_12] = min (abs(freq(1:250,:)-12));
        
        [~,minloc_30] = min (abs(freq(1:250,:)-30));
        
        [~,minloc_80] = min (abs(freq(1:250,:)-80));
        
        
        % Delta
        vec_PSD_delta= sum(psd_2(minloc_05:minloc_4,:),1);
        % Theta
        vec_PSD_theta= sum(psd_2(minloc_4:minloc_8,5:10),1);
        % Alpha
        vec_PSD_alpha= sum(psd_2(minloc_8:minloc_12,5:10),1);
        % Beta
        vec_PSD_beta = sum(psd_2(minloc_12:minloc_30,5:10),1);
        % Gamma
        vec_PSD_gamma= sum(psd_2(minloc_30:minloc_80,5:10),1);
        
        %caracteristicas completas
        vec_feat = [prom,vec_PSD_delta,vec_PSD_theta,vec_PSD_alpha,vec_PSD_beta,vec_PSD_gamma,vec_psd,vari,med,moda];
        
        tags_modelo_nivel1(contador_t) = Modelo_Voluntario_1.predictFcn(vec_feat);
        
         
    end %ventana
    
        
end

sim.delete(); % finaliza la comunicacion.
disp('Program ended');


tags_modelo_nivel1(end-10:end) = [];
%guardar variable
tags_modelo_nivel1;

%predicciones... Matriz de confusión

Correcto_n1=[1,1,1,2,2,2,1,1];
ctn_n1=1;
vec_corr_n1 =[];
for f=1:length(tags_modelo_nivel1)
    if Correcto_n1(ctn_n1)==tags_modelo_nivel1(f)
        vec_corr_n1(f)=Correcto_n1(ctn_n1);
        ctn_n1=ctn_n1+1;
    else
        vec_corr_n1(f)=Correcto_n1(ctn_n1);
    end 
end

Confus_n1 = confusionmat(vec_corr_n1,tags_modelo_nivel1)


gds_interface.StopDataAcquisition(); %detener adquisicion
delete(gds_interface);
clear gds_interface;