classdef LPF2 < handle
   properties
      hz
      dt
      alpha
      beta
      omega
      freq
   end
   methods
      function obj = set_dt(obj, dt)
          obj.dt = dt;
          obj.hz = 1/dt;
      end
      function obj = set_hz(obj, hz)
          obj.hz = hz;
          obj.dt = 1/hz;
      end
      
      function obj = set_alpha(obj, alpha)
          obj.alpha = alpha;
          obj.beta = 1-alpha;
      end
      function obj = set_beta(obj, beta)
          obj.beta = beta;
          obj.alpha = 1-beta;
      end
      
      function obj = set_omega(obj, omega)
          obj.omega = omega;
          obj.freq = omega/(2*pi);
      end
      function obj = set_freq(obj, freq)
          obj.freq = freq;
          obj.omega = 2*pi*freq;
      end
      
      function alpha = get_alpha(obj)
          obj.set_beta(1/(1+obj.omega * obj.dt));
          alpha = obj.alpha;
      end
      function beta = get_beta(obj)
          obj.set_beta(1/(1+obj.omega * obj.dt));
          beta = obj.beta;
      end
      
      function omega = get_omega(obj)
          obj.set_omega((1-obj.beta) / (obj.beta*obj.dt));
          omega = obj.omega;
      end
      function freq = get_freq(obj)
          obj.set_omega((1-obj.beta) / (obj.beta*obj.dt));
          freq = obj.freq;
      end
   end
end