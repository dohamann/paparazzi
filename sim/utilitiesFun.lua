-- p, i and d: coefficients for proportional, integral and derivative
-- errorScaleCoeff and pidScaleCoeff: use to scale floats to integers by preserving different precision values
  -- precision is retained according to the magnitude of the scaling coefficient, e.g:
  -- p = 0.1234, pidScaleCoeff = 10 	-> assume p = 0.1
  -- p = 0.1234, pidScaleCoeff = 100 	-> assume p = 0.12
  -- p = 0.1234, pidScaleCoeff = 10000 	-> assume p = 0.1234
function newPIDController(p, i, d, errorScaleCoeff, pidScaleCoeff)
  
	local integral = 0	-- current sum of errors, scaled
	local previousError = 0 -- last error, scaled
	
	local min = 0	-- allowed minimal output value, scaled
	local max = 0	-- allowed maximal output value, scaled
	local checkMinMax = 0	-- are min/max boundaries set
	
	local _UPDATE_RATE_COEFF =  getUpdateRateCoeff()	-- SIMULATION ONLY: consider update rate adjustments
	local timeStep = simGetSimulationTimeStep() * _UPDATE_RATE_COEFF
	
	-- scaled P,I and D coefficients from float to int
	local pS = getAsIntScaled(p, pidScaleCoeff)
	local iS = getAsIntScaled(i, pidScaleCoeff)
	local dS = getAsIntScaled(d, pidScaleCoeff)
	
	-- sets scaled min and max boundaries
	-- also sets i-coefficient to 0 in order to avoid integral windup
	function setMinMax(minParam, maxParam)
	    min = getAsIntScaled(minParam, errorScaleCoeff*pidScaleCoeff)
	    max = getAsIntScaled(maxParam, errorScaleCoeff*pidScaleCoeff)
	    iS = 0
	    checkMinMax = 1
	end
	
	-- reset the controller
	function clear()
	    integral = 0
	    previousError = 0
	end
	
	-- scaling is carried out internally, return value is in the same scale as input
	function adjust(error)
		
		local errorS = getAsIntScaled(error, errorScaleCoeff) -- scale error
		
		integral = integral + (errorS )
		local derivative = 0

		if (previousError ~= 0) then	-- ignore error derivative on initialisation step
			derivative = (errorS - previousError) / timeStep
		end
		
		previousError = errorS

		local res = (pS * errorS) + (iS * integral * timeStep) + (dS * derivative)
		
		--res = max *res / math.sqrt(1 + res*res)
		
		if(checkMinMax == 1) then -- do we need sigmoid to soften min/max clipping for discrete values?
		  if(res < min) then
		    res = min
		  end
		  if (res > max) then
		    res = max
		  end
		end
		
		-- scale the result back
		return getRealValue(res, errorScaleCoeff, pidScaleCoeff)
	end

	return {
		setMinMax = setMinMax,
		adjust = adjust,
		clear = clear
	}
end


-- controller with predefined scaling coefficients
function newDirectDistToAngleController(pidCoeff, minDist)
  
  local daPID = newPIDController(pidCoeff[1], pidCoeff[2], pidCoeff[3], getScaleCoeff(), getScalePIDCoeff())
  
  function setMinMax(min, max)
    daPID.setMinMax(min,max)
  end
  
  function adjust(dist)
    return daPID.adjust(minDist - dist)
  end
  
  function clear()
    daPID.clear()
  end
  
  return {
    setMinMax = setMinMax,
    adjust = adjust,
    clear = clear
  }
end


-- SIMULATION ONLY: use to simulate lower update rate (simulated frequency = 20Hz / _UPDATE_RATE_COEFF)
function getUpdateRateCoeff()
  return 5
end

-- central source for scaling coefficients: change the return value to globally increase/decrease precision
function getScaleCoeff()
  return 1000
end

function getScalePIDCoeff()
  return 10
end

-- returns floored value
function getAsInt(feed)
  return math.floor(feed)
end

-- scales input value and returns integer part, e.g:
  -- feed = 0.1234, scaleCoeff = 10 	-> 1
  -- feed = 0.1234, scaleCoeff = 100 	-> 12
  -- feed = 0.1234, scaleCoeff = 10000 	-> 1234
function getAsIntScaled(feed, scaleCoeff)
  return math.floor(feed * scaleCoeff)
end

-- scales input back according to error and pid scales
function getRealValue(feed, errorScaleCoeff, pidScaleCoeff)
  return feed / (errorScaleCoeff*pidScaleCoeff)
end

