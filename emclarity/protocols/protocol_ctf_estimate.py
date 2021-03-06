# -*- coding: utf-8 -*-
# **************************************************************************
# *
# * Authors:     you (you@yourinstitution.email)
# *
# * your institution
# *
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation; either version 2 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program; if not, write to the Free Software
# * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# * 02111-1307  USA
# *
# *  All comments concerning this program package may be sent to the
# *  e-mail address 'you@yourinstitution.email'
# *
# **************************************************************************


"""
Describe your python module here:
This module will provide the traditional Hello world example
"""
import os

from pyworkflow.protocol import Protocol

import imod.utils as utils
import numpy as np
import pwem.objects as data
from pwem.objects import Transform
from pyworkflow import BETA
from pyworkflow.object import Set, Integer
import pyworkflow.protocol.params as params
import tomo.objects as tomoObj
from emclarity import Plugin


PARAMS_FN = 'param_ctf.m'

class ProtEmclarityCtfEstimate(Protocol):
    """
    This protocol will execute the protocol ctf estimate calling emClarity program
    """
    _label = 'ctf estimate'

    # -------------------------- DEFINE param functions ----------------------
    def _defineParams(self, form):
        """ Define the input parameters that will be used.
        Params:
            form: this is the form to be populated with sections and params.
        """
        # You need a params to belong to a section:
        form.addSection('Input')

        form.addParam('inputSetOfTiltSeries',
                      params.PointerParam,
                      pointerClass='SetOfTiltSeries',
                      important=True,
                      label='Input set of tilt-series.')

        form.addParam('VOLTAGE', params.FloatParam,
                      default=300e3,
                      important=True,
                      label='VOLTAGE (V)',
                      help='Accelerating voltage of the microscope in Volts')

        form.addParam('Cs', params.FloatParam,
                      default=2.7e-3,
                      label='Cs (m)',
                      important=True,
                      help='Spherical aberration of the microscope in meters')

        form.addParam('AMPCONT', params.FloatParam,
                      default=0.10,
                      label='AMPCONT',
                      important=True,
                      help='Percent amplitude contrast ratio')

        form.addParam('SuperResolution', params.FloatParam,
                      default=0,
                      label='SuperResolution',
                      important=True,
                      help='Whether the stacks are super-sampled.'
                           'If 1, emClarity will Fourier crop by a factor of 2 and'
                           'set the actual pixel size to 2 * PIXEL_- SIZE.' 
                           'Note that this is not tested anymore,' 
                           'so it is preferable to Fourier crop the stacks beforehand and set it to 0')

        form.addParam('beadDiameter', params.FloatParam,
                      default=10e-9,
                      label='Bead Diameter (m)',
                      help='Diameter of the beads to erase, in meters (e.g. 10e-9). This'
                           'parameter is used if fiducial beads need to be erased, thus only'
                           'for stacks with a fixedStacks/*.erase file.')

        form.addParam('erase_beads_after_ctf', params.BooleanParam,
                      default=False,
                      label='Erase beads after ctf',
                      help='Whether or not the fiducial beads should be removed on the raw'
                           'tilt-series (now) or on the CTF multiplied tilt-series computed during the tomogram'
                           'reconstruction (section 10). Do not change this'
                           'option between ctf estimate and ctf 3d')

        form.addParam('CUM_e_DOSE', params.FloatParam,
                      default=60,
                      label='CUM e DOSE (e/??2)',
                      important=True,
                      help='Total exposure in e/??2')

        form.addParam('doseAtMinTilt', params.FloatParam,
                      default=1.46,
                      label='Dose At Min Tilt (e/??2)',
                      important=True,
                      help='The exposure each view receive (should be about CUM_e_DOSE'
                           'nb of views), in e/??2')

        form.addParam('oneOverCosineDose', params.FloatParam,
                      default=0,
                      label='One Over Cosine Dose',
                      important=True,
                      help='Whether or not it is a Saxton scheme (dose increase as 1/cos(??),'
                           '?? being the tilt angle); this will scale doseAtMinTilt according'
                           'to the tilt angle (e.g. 0)')

        form.addParam('startingAngle', params.FloatParam,
                      default=30,
                      label='Starting Angle (degrees)',
                      important=True,
                      help='Starting angle, in degrees')

        form.addParam('startingDirection', params.StringParam,
                      default='neg', 
                      label='Starting Direction',
                      important=True,
                      help='Starting direction; should the angles decrease or increase (neg or pos)')

        form.addParam('doseSymmetricIncrement', params.FloatParam,
                      default=0,
                      label='Dose Symmetric Increment',
                      important=True,
                      help='The number of tilts before each switch in direction. 0=false,'
                           '2="normal" dose symmetric. The original dose symmetric scheme'
                           'included 0 in the first group. For this, specify the number as'
                           'a negative number')

        form.addParam('defCutOff', params.FloatParam,
                      default=7e-10,
                      label='defCutOff (m)',
                      important=True,
                      help='The power spectrum used by ctf estimate is considered from'
                           'slightly before the first zero past the first zero to this' 
                           'cutoff, in meter')

        form.addParam('defEstimate', params.FloatParam,
                      default=3e-6, 
                      label='defEstimate (m)',
                      important=True,
                      help='Initial rough estimate of the defocus, in meter. With defWindow,'
                           'it defines the search window of defoci')

        form.addParam('defWindow', params.FloatParam,
                      default=1.5e-6,
                      label='defWindow (m)',
                      important=True,
                      help='Defocus window around defEstimate, in meter;'
                       'e.g. if defEstimate = 2.5e-6 and defWindow = 1.5e-6,'
                        'try a range of defocus between 1e-6 to 4e-6')

        form.addParam('deltaZtolerance', params.FloatParam,
                      default=50e-9,
                      label='Delta Z Tolerance (m)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Includes the tiles with defocus equal to'
                           'that at the tilt-axis ?????Z,in meters')

        form.addParam('zShift', params.FloatParam,
                      default=150e-9,
                      label='Z Shift (m)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Used for the handedness check. Shift the evaluation region'
                           '(Ztilt???axis ?? deltaZtolerance) by this amount')

        form.addParam('ctfMaxNumberOfTiles', params.FloatParam,
                      default=4000,
                      label='Ctf Max Number Of Tiles',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Limits the number of tiles to include in the power spectrum.'
                           'The more tiles, the stronger the signal but the longer it takes to'
                           'compute the power spectrum')

        form.addParam('ctfTileSize', params.FloatParam,
                      default=680e-10,
                      label='Ctf Tile Size (m)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Size of the (square) tiles, in meters')

        form.addParam('paddedSize', params.FloatParam,
                      default=768,
                      label='Padded Size (px)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='The tiles are padded to this size, in pixel,'
                           'in real space before computing the Fourier transform.'
                           'Should be even, large (compared to the tiles),' 
                           'and preferably a power of 2')


    # --------------------------- STEPS functions ------------------------------
    def _insertAllSteps(self):
        for ts in self.inputSetOfTiltSeries.get():
            self._insertFunctionStep(self.ctf_estimateStep, ts.getObjId())
        #self._insertFunctionStep(self.closeOutputSetsStep)

    def ctf_estimateStep(self, tsObjId):
        """Compute the alignment of the tilt series"""
        ts = self.inputSetOfTiltSeries.get()[tsObjId]
        tsId = ts.getTsId()
        workingDir = self._getExtraPath(tsId)

        import pyworkflow.utils.path as path
        pathToFixedStacks = os.path.join(workingDir,'fixedStacks')
        path.makePath(pathToFixedStacks)

        print(workingDir)
        print(ts.getFirstItem().parseFileName())

        tlt = os.path.join(pathToFixedStacks, tsId+'.tlt')
        ts.generateTltFile(tlt)
        xf = os.path.join(pathToFixedStacks, tsId+'.xf')
        ts.writeXfFile(xf)

        stack = os.path.abspath(ts.getFirstItem().getFileName())
        symbLinkToTS = os.path.join(workingDir, 'fixedStacks', tsId+'.fixed')
        #relpathToTS = os.path.relpath(symbLinkToTS, workingDir)

        os.symlink(stack, symbLinkToTS)

        sampling = ts.getSamplingRate()
        self.create_parameters_file(sampling, workingDir)

        argsCtf_estimate = "%s" %PARAMS_FN
        argsCtf_estimate += " %s" % tsId

        print(argsCtf_estimate)
        Plugin.runEmClarity(self, 'ctf estimate', argsCtf_estimate, cwd=self._getExtraPath(tsId))

    def create_parameters_file(self, sampling, workingDir):
        f = open(os.path.join(workingDir, PARAMS_FN), 'w')
        pixel_size = sampling * 1e-10
        f.write('nGPUs=1')
        f.write('\nnCpuCores=1')
        f.write('\nPIXEL_SIZE=' + str(pixel_size))
        f.write('\nSuperResolution=' + str(self.SuperResolution))
        f.write('\nCs=' + str(float(self.Cs)))
        f.write('\nVOLTAGE=' + str(float(self.VOLTAGE)))
        f.write('\nAMPCONT=' + str(float(self.AMPCONT)))
        f.write('\ndefEstimate=' + str(self.defEstimate))
        f.write('\ndefWindow=' + str(self.defWindow))
        f.write('\ndefCutOff=' + str(self.defCutOff))
        f.write('\nCUM_e_DOSE=' + str(self.CUM_e_DOSE))
        f.write('\nbeadDiameter=' + str(self.beadDiameter))
        f.write('\noneOverCosineDose=' + str(self.oneOverCosineDose))
        f.write('\nstartingAngle=' + str(self.startingAngle))
        f.write('\nstartingDirection=' + str(self.startingDirection))
        f.write('\ndoseSymmetricIncrement=' + str(self.doseSymmetricIncrement))
        f.write('\ndoseAtMinTilt=' + str(self.doseAtMinTilt))
        f.close()
