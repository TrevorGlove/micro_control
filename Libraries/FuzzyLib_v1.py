# FuzzyLib v1.0 por Robert Vásquez
# @Trevor Glove 

import ulab.numpy as np
import math as mt
from math import pi

# Membership Functions

def trapmf(x, params):
    a, b, c, d = params  
    if a <= x < b:
        return (x - a) / (b - a)
    elif b <= x <= c:
        return 1
    elif c < x <= d:
        return (d - x) / (d - c)
    else:
        return 0
    
def trimf(x, params):
    a, b, c = params
    return np.maximum(np.minimum((x - a) / (b - a), (c - x) / (c - b)), 0)
       
def sigmf(x, params):
    ak, ck = params
    return 1 / (1 + np.exp(-ak * (x - ck))) 

def zmf(x, params):
    a, b = params
    if x <= a:
        return 1
    elif a <= x <= (a + b) / 2:
        return 1 - 2 * mt.pow((x - a) / (b - a), 2)
    elif (a + b) / 2 <= x <= b:
        return 2 * mt.pow((x - b) / (b - a), 2)
    else:
        return 0

def gauss2mf(x, params):
    var, mean = params
    return np.exp(-mt.pow(x - mean, 2) / (2 * mt.pow(var, 2)))

# Membership Inverse Functions

def itrapmf(y, params):
    if y < 0.0 or y > 1.0:
        return None
    
    elif 0.0 <= y <= 1.0:
        a, b, c, d = params
        
        if y == 0.0:
            return a, d  
        elif y == 1.0:
            return b, c  
        elif 0 < y < 1 and a != b and c != d:
            x1 = a + (b - a)*y
            x2 = d - (d - c)*y
            return x1, x2
        elif 0 < y < 1 and a == b and c != d:
            x1 = a
            x2 = d - (d - c)*y
            return x1, x2
        elif 0 < y < 1 and a != b and c == d:
            x1 = a + (b - a)*y
            x2 = d
            return x1, x2
        elif 0 < y < 1 and a == b and c == d:
            return a, d
        elif 0 < y < 1 and b == c:
            x1 = a + (b - a)*y
            x2 = d - (d - c)*y
            return a, d
        else:
            return None
    else:
        return None

def itrimf(y, params):
    if y < 0.0 or y > 1.0:
        return None  
    
    a, b, c = params
    x1 = a + (b - a) * y
    x2 = c - (c - b) * y
    
    return x1, x2

    
class fuzzy_membership:
    def __init__(self, params, membership_func):
        self._params = params
        self._membership_func = membership_func
        self._fuzzify_rules = None
        self._fuzzify_vrules = None
        self.fuzzify_value = None
        self.area = None
        self.centroid = None
        
    @property
    def fuzzify_rules(self):
        return self._fuzzify_rules
    
    @fuzzify_rules.setter
    def fuzzify_rules(self, value):
        self._fuzzify_rules = value
            
    @property
    def fuzzify_vrules(self):
        return self._fuzzify_vrules
    
    @fuzzify_vrules.setter
    def fuzzify_vrules(self, value):
        self._fuzzify_vrules = value
        self.operate_rule()
        self.compute_centroid()
        
    def operate_rule(self):     
        self.fuzzify_value = np.max(self._fuzzify_vrules)

    def value(self, x):
        if self._membership_func == 'trapezoidal':
            return trapmf(x, self._params)
        elif self._membership_func == 'triangular':
            return trimf(x, self._params)
        elif self._membership_func == 'sigmoidal':
            return sigmf(x, self._params)
        elif self._membership_func == 'zshaped':
            return zmf(x, self._params)
        elif self._membership_func == 'zshaped linear':
            return linzmf(x, self._params)
        elif self._membership_func == 'gaussian':
            return gauss2mf(x, self._params)
        else:
            raise ValueError("Función de membresía no válida")
            
    def compute_centroid(self):
        
        if self.fuzzify_value == 0:
            self.centroid = None
            
        else:
            if self._membership_func == 'trapezoidal':
                a, b, c, d = self._params
                m, n = itrapmf(self.fuzzify_value, self._params)
             
                A1 = self.fuzzify_value*(m - a)/2
                xc1 = (2*m + a)/3
             
                A2 = self.fuzzify_value*(n - m)
                xc2 = (m + n)/2
             
                A3 = self.fuzzify_value*(d - n)/2
                xc3 = (d + 2*n)/3
             
                self.area = A1 + A2 + A3
                self.centroid = (A1*xc1 + A2*xc2 + A3*xc3)/self.area 
          
            elif self._membership_func == 'triangular':
                a, b, c = self._params
                m, n = itrimf(self.fuzzify_value, self._params)
             
                A1 = self.fuzzify_value*(m - a)/2
                xc1 = (2*m + a)/3
             
                A2 = self.fuzzify_value*(n - m)
                xc2 = (m + n)/2
             
                A3 = self.fuzzify_value*(c - n)/2
                xc3 = (c + 2*n)/3
             
                self.area = A1 + A2 + A3
                self.centroid = (A1*xc1 + A2*xc2 + A3*xc3)/self.area
            else:
                raise ValueError("Operador difuso no válido")

class fuzzyficator:
    def __init__(self):
        self._membership_in = None
        self._membership_out = None
        self._entry = None
        
    @property
    def rules(self):
        return self._rules
    
    @rules.setter
    def rules(self, value):
        self._rules = value
    
    @property
    def membership_out(self):
        return self._membership_out
    
    @membership_out.setter
    def membership_out(self, value):
        self._membership_out = value
    
    @property
    def entry(self):
        return self._entry
    
    @entry.setter
    def entry(self, value):
        self._entry = value
        self.operate_rule()
        
    def defuzzy(self):
        xc = []
        A = []
        
        for obj in self._membership_out[0]:
            if obj.centroid is not None and obj.area is not None:
                xc.append(obj.centroid)
                A.append(obj.area)
        
        xc = np.array(xc)
        A = np.array(A)
        
        if np.sum(A) == 0:
            return None
        
        out = np.sum(xc * A) / np.sum(A)
        
        return out
    
    def operate_rule(self):
        
        for obj in self._membership_out[0]:
            
            tam = len(obj.fuzzify_rules)
            values = np.zeros(tam)
            
            for k in range(0,tam):
                funct1 = obj.fuzzify_rules[k][0]

                funct2 = obj.fuzzify_rules[k][1]
                operator = obj.fuzzify_rules[k][2]
                
                if operator == 'and':
                    m = funct1.value(self._entry[0])
                    n = funct2.value(self._entry[1])
                    values[k-1] = np.min([m,n])
                elif operator == 'or':
                    m = funct1.value(self._entry[0])
                    n = funct2.value(self._entry[1])
                    values[k-1] = np.max([m,n])
                else:
                    raise ValueError("Operador difuso no válido")
            
            obj.fuzzify_vrules = values
