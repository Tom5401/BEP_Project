number_of_tests=4;          %amount of files
m=1.630;                    %mass of the car
fit_type="poly2";           %type "poly1" for a first order polynomial fit
graph_steps_between=0;      %change to 1 in order to graph the steps inbetween

CumulativeC1=0;
CumulativeC2=0;
CumulativeC3=0;

for n=1:number_of_tests
    filename="coastdown"+n+".csv";
    time=cell2mat(readcell(filename,"Range",'B8:B5000'));               %Time list
    x_pos=cell2mat(readcell(filename,"Range",'G8:G5000'));              %X position list
    y_pos=cell2mat(readcell(filename,"Range",'H8:H5000'));              %Y position list, wont be used as that should be zero
    z_pos=cell2mat(readcell(filename,"Range",'I8:I5000'));              %Z position list
    Fs=cell2mat(readcell(filename,"Range",'H1:H1'));                    %measurement frequency in Hz
    Total_frames_exported=cell2mat(readcell(filename,"Range",'N1:N1')); %total amount of measurements
    dt=1/Fs;                                                            %timestep between measurements
    
    %using symetric difference to determine the velocity
    vel=zeros(Total_frames_exported,1);
    for n = 2 : Total_frames_exported-1                                 %Used the -1 because otherwise the function will act up at the last timestep
        x_vel=(x_pos(n+1)-x_pos(n-1))/(2*dt);
        z_vel=(z_pos(n+1)-z_pos(n-1))/(2*dt);
        vel(n)=sqrt((x_vel)^2+(z_vel)^2);
    end
    vel(1)=vel(2);
    vel(end)=vel(end-1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if graph_steps_between==1
        figure(1)
        plot(linspace(0,length(vel)*dt,length(vel)),vel)
        title("velocity from optitrack data determined using symmetric difference")
        hold on;
    else
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %filtering out small peak in velocity from when the jetracer enters the optitrack area
    for n = 1 : Total_frames_exported
        if vel(n)~=0
            Frame_0=n;                                                   %when the measured velocity deviates from zero, the car has entered the optitrack area
            break
        end
    end
    vel_without_initial_peak=[vel(Frame_0+10:end)];                      %adding a small margin to smooth out initial extremes in velocity
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if graph_steps_between==1
        figure(2)
        plot(linspace(0,length(vel_without_initial_peak)*dt,length(vel_without_initial_peak)),vel_without_initial_peak)
        title("Velocity without the peak when the car enters the optitrack area")
        hold on;
    else
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %using lowpass filter to smooth velocity
    lowpass_filtered_vel = lowpass(vel_without_initial_peak,0.01,1/dt);
    
    %determining the start and end index of the deceleration part of the velocity curve
    [maximum_velocity,start]=max(lowpass_filtered_vel);
    "maximum velocity before coastdown is: "+maximum_velocity;
    for n = 1 : Total_frames_exported
        if lowpass_filtered_vel(n)<=0.05
            stop=n;
            break
        end
    end
    filtered_vel_short=[lowpass_filtered_vel(start+10:stop)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if graph_steps_between==1
        figure(3)
        plot(linspace(0,length(filtered_vel_short)*dt,length(filtered_vel_short)),filtered_vel_short)
        title("Velocity with a low pass filter to filter out noise")
        hold on;
    else
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %using numerical differentiaton (forward difference) to determine the
    %acceleration
    acc=zeros(length(filtered_vel_short),1);
    for n = 2 : length(acc)-1
        acc(n)=(filtered_vel_short(n+1)-filtered_vel_short(n-1))/(2*dt);
    end
    acc(1)=acc(2);
    acc(end)=acc(end-1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if graph_steps_between==1
        figure(4)
        plot(linspace(0,length(acc)*dt,length(acc)),acc)
        title("acceleration of the car during coast down")
        hold on;
    else
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %fitting a first order polynomial to the curve
    linear_fit=fit(filtered_vel_short,-acc*m,fit_type);
    if fit_type=="poly2"
        CumulativeC1=CumulativeC1+linear_fit.p1;
        CumulativeC2=CumulativeC2+linear_fit.p2;
        CumulativeC3=CumulativeC3+linear_fit.p3;
    else
        CumulativeC1=CumulativeC1+linear_fit.p1;
        CumulativeC2=CumulativeC2+linear_fit.p2;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(5)
    hold on;
    plot(filtered_vel_short,-acc*m)
    %plot(linear_fit)
    title("Combined drag and rolling resistance acting on the car as function of velocity")
    xlabel("Velocity in m/s")
    ylabel("Force in N")
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end
C1_average=CumulativeC1/number_of_tests;
C2_average=CumulativeC2/number_of_tests;
C3_average=CumulativeC3/number_of_tests;

%Display answers
if fit_type=="poly2"
    disp("the second order polynomial fit is F = "+C1_average+"*v^2 + "+C2_average+"*v + "+C3_average)
    v=linspace(0,3.3,500);
    f=C1_average*v.^2+C2_average*v+C3_average;
    disp("coefficient 1 = "+C1_average)
    disp("coefficient 2 = "+C2_average)
    disp("coefficient 3 = "+C3_average)
else
    disp("the first order polynomial fit is F = "+C1_average+"*v + "+C2_average)
    v=linspace(0,3.3,500);
    f=C1_average*v+C2_average;
    disp("coefficient 1 = "+C1_average)
    disp("coefficient 2 = "+C2_average)
end
figure(5)
grid on
plot(v,f,"LineWidth",2)