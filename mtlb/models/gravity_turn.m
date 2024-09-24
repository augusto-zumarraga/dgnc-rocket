% velocidad angular
function q = gravity_turn(V, R, y)
    g = 3.986004418000000e+014 / R^2
    q = (V/R - g/V) * cos(y);  
end

