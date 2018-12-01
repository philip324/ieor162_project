function route_map = forward_sweep(final_VDC,center_dealer,...
    dealer_vehicle_map,location)

shift = @(long) mod(long+360,360)-180;
vdc_loc = get_location(final_VDC,location);
dealers = cell2mat(keys(dealer_vehicle_map)');

center_loc = get_location(center_dealer,location);
dy = center_loc(1) - vdc_loc(1);
dx = shift(center_loc(2)) - shift(vdc_loc(2));
offset_angle = atan2d(dy,dx);
angles = [];
for i = 1:length(dealers)
    dea_loc = get_location(dealers(i),location);
    dy = dea_loc(1) - vdc_loc(1);
    dx = shift(dea_loc(2)) - shift(vdc_loc(2));
    
    candidate1 = atan2d(dy,dx) - offset_angle;
    candidate2 = mod(candidate1+720,720)-360;
    if abs(candidate1) < abs(candidate2)
        angles(i) = candidate1;
    else
        angles(i) = candidate2;
    end
end

route_map = containers.Map('KeyType','double','ValueType','any');
total_veh = size(dealer_vehicle_map(center_dealer),1);
if total_veh >= 10
    vehicles = dealer_vehicle_map(center_dealer);
    vehicles = vehicles(10,:);
    route_map(center_dealer) = vehicles;
    return;
end

idx = find(dealers == center_dealer);
angles(idx) = [];
dealers(idx) = [];

route_map(center_dealer) = dealer_vehicle_map(center_dealer);
while total_veh < 10
    min_angle = Inf;
    min_idx = 0;
    for i = 1:length(angles)
        if abs(angles(i)) < abs(min_angle)
            min_angle = angles(i);
            min_idx = i;
        end
    end
    
    closest_dealer = dealers(min_idx);
    angles(min_idx) = [];
    dealers(min_idx) = [];
    if total_veh+size(dealer_vehicle_map(closest_dealer),1) <= 10
        route_map(closest_dealer) = dealer_vehicle_map(closest_dealer);
        total_veh = total_veh + size(dealer_vehicle_map(closest_dealer),1);
    else
        vehicles = dealer_vehicle_map(closest_dealer);
        vehicles = vehicles(10-total_veh,:);
        route_map(closest_dealer) = vehicles;
        total_veh = 10;
    end
end
end