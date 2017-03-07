function [V] = voltageModel(u2)
%model to convert channel input to Volage for motor

duty=(u2)/500;
V=5.6*duty;

end

