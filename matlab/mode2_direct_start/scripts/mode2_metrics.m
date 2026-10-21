function m=mode2_metrics(r)
t=r.time; y=r.y; q=r.q;
tail=t>=q.stop_time-0.2;
m.final_actual_rpm=mean(y(tail,12)); m.final_estimated_rpm=mean(y(tail,3));
m.final_actual_target_error_rpm=m.final_actual_rpm-q.third_rpm;
m.final_estimated_target_error_rpm=m.final_estimated_rpm-q.third_rpm;
m.final_actual_speed_std_rpm=std(y(tail,12));
m.tail_speed_estimation_RMSE_rpm=sqrt(mean((y(tail,3)-y(tail,12)).^2));
angle=atan2(sin(y(:,1)-y(:,13)),cos(y(:,1)-y(:,13)));
m.tail_angle_RMSE_deg=sqrt(mean(angle(tail).^2))*180/pi;
m.peak_phase_A=max(abs(y(:,15:17)),[],'all');
m.peak_Iq_reference_A=max(abs(y(:,22)));
m.peak_actual_speed_rpm=max(abs(y(:,12)));
m.ccr_min=min(y(:,28:30),[],'all'); m.ccr_max=max(y(:,28:30),[],'all');
m.tracks_final_target=double(abs(m.final_actual_target_error_rpm)<max(20,0.05*q.third_rpm) && ...
    m.tail_speed_estimation_RMSE_rpm<50 && m.final_actual_speed_std_rpm<50);
windows=[q.switch_1 q.switch_2 q.stop_time];
for k=1:3
    idx=t>=windows(k)-0.15 & t<windows(k);
    m.(sprintf('stage%d_actual_rpm',k))=mean(y(idx,12));
    m.(sprintf('stage%d_estimated_rpm',k))=mean(y(idx,3));
end
end
