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


PARAMS_FN = 'param.m'
RAWTLT_FN = 'tilt.rawtlt'

class ProtEmclarityCtfEstimate(Protocol):
    """
    This protocol will print hello world in the console
    IMPORTANT: Classes names should be unique, better prefix them
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
                      default=2.7e-6,
                      label='Cs (m)',
                      important=True,
                      help='Spherical aberration of the microscope in meters')

        form.addParam('AMPCONT', params.FloatParam,
                      default=0.09,
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
                      default=4.0,
                      label='CUM e DOSE (e/Å2)',
                      important=True,
                      help='Total exposure in e/Å2')

        form.addParam('doseAtMinTilt', params.FloatParam,
                      default=64.0,
                      label='Dose At Min Tilt (e/Å2)',
                      important=True,
                      help='The exposure each view receive (should be about CUM_e_DOSE'
                           'nb of views), in e/Å2')

        form.addParam('oneOverCosineDose', params.FloatParam,
                      default=0,
                      label='One Over Cosine Dose',
                      important=True,
                      help='Whether or not it is a Saxton scheme (dose increase as 1/cos(α),'
                           'α being the tilt angle); this will scale doseAtMinTilt according'
                           'to the tilt angle (e.g. 0)')

        form.addParam('startingAngle', params.FloatParam,
                      default=0,
                      label='Starting Angle (degrees)',
                      important=True,
                      help='Starting angle, in degrees')

        form.addParam('startingDirection', params.StringParam,
                      default='', # neg o pos ?
                      label='Starting Direction',
                      important=True,
                      help='Starting direction; should the angles decrease or increase (neg or pos)')

        form.addParam('doseSymmetricIncrement', params.FloatParam,
                      default=2, # Dice algo de especificar el numero como negativo
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
                      default=0, # ni idea que poner
                      label='defEstimate (m)',
                      important=True,
                      help='Initial rough estimate of the defocus, in meter. With defWindow,'
                           'it defines the search window of defoci')

        form.addParam('defWindow', params.FloatParam,
                      default=1.5e-6,
                      label='defWindow (m)',
                      important=True,
                      help='Starting angle, in degrees')

        form.addParam('startingAngle', params.FloatParam,
                      default=0,
                      label='Starting Angle (degrees)',
                      important=True,
                      help='Starting angle, in degrees')

        form.addParam('deltaZtolerance', params.FloatParam,
                      default=50e-9,
                      label='Delta Z Tolerance (m)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Includes the tiles with defocus equal to'
                           'that at the tilt-axis ±∆Z,in meters')

        form.addParam('zShift', params.FloatParam,
                      default=0,
                      label='Z Shift (??)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Used for the handedness check. Shift the evaluation region'
                           '(Ztilt−axis ± deltaZtolerance) by this amount')

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
            #self._insertFunctionStep(self.convertInputStep, ts.getObjId())
            self._insertFunctionStep(self.autoAlignStep, ts.getObjId())
        #     self._insertFunctionStep(self.generateOutputStackStep, ts.getObjId())
        #     if self.computeAlignment.get() == 0:
        #         self._insertFunctionStep(self.computeInterpolatedStackStep, ts.getObjId())
        # self._insertFunctionStep(self.closeOutputSetsStep)

    def autoAlignStep(self, tsObjId):
        """Compute the alignment of the tilt series"""
        ts = self.inputSetOfTiltSeries.get()[tsObjId]
        tsId = ts.getTsId()
        extraPrefix = self._getExtraPath(tsId)

        import pyworkflow.utils.path as path
        path.makePath(extraPrefix)

        tsPath = os.path.join(extraPrefix, ts.getFirstItem().getFileName())
        tmpPrefix = self._getTmpPath(tsId)
        print(extraPrefix)
        print(ts.getFirstItem().parseFileName())

        #stack = os.path.join(extraPrefix, ts.getFirstItem().parseFileName())
        stack = os.path.abspath(ts.getFirstItem().getFileName())
        #rawtlt = os.path.join(extraPrefix, ts.getFirstItem().parseFileName(extension=".prexf"))
        rawtlt = self._getExtraPath(RAWTLT_FN)
        ts.generateTltFile(rawtlt)
        rotationAngle = ts.getAcquisition().getTiltAxisAngle()

        sampling = ts.getSamplingRate()
        param_file = self.create_parameters_file(sampling)

        argsAutoAlign = "%s" %PARAMS_FN
        argsAutoAlign += " %s" %stack
        argsAutoAlign += " %s" %RAWTLT_FN
        argsAutoAlign += " %s" %rotationAngle

        print(argsAutoAlign)
        Plugin.runEmClarity(self, 'autoAlign', argsAutoAlign, cwd=self._getExtraPath())


    def generateOutputStackStep(self):
        # TO DO ADAPT FUNCTION FROM XCORR
        """ Generate tilt-serie with the associated transform matrix """
        ts = self.inputSetOfTiltSeries.get()
        tsId = ts.getTsId()

        extraPrefix = self._getExtraPath(tsId)

        self.getOutputSetOfTiltSeries(self.inputSetOfTiltSeries.get())

        alignmentMatrix = utils.formatTransformationMatrix(
            os.path.join(extraPrefix, ts.getFirstItem().parseFileName(extension=".prexg")))

        newTs = tomoObj.TiltSeries(tsId=tsId)
        newTs.copyInfo(ts)

        self.outputSetOfTiltSeries.append(newTs)

        for index, tiltImage in enumerate(ts):
            newTi = tomoObj.TiltImage()
            newTi.copyInfo(tiltImage, copyId=True, copyTM=False)

            if tiltImage.hasTransform():
                transform = Transform()
                previousTransform = tiltImage.getTransform().getMatrix()
                newTransform = alignmentMatrix[:, :, index]
                previousTransformArray = np.array(previousTransform)
                newTransformArray = np.array(newTransform)
                outputTransformMatrix = np.matmul(newTransformArray, previousTransformArray)
                transform.setMatrix(outputTransformMatrix)
                newTi.setTransform(transform)

            else:
                transform = Transform()
                newTransform = alignmentMatrix[:, :, index]
                newTransformArray = np.array(newTransform)
                transform.setMatrix(newTransformArray)
                newTi.setTransform(transform)

            newTi.setAcquisition(tiltImage.getAcquisition())
            newTi.setLocation(tiltImage.getLocation())

            newTs.append(newTi)

        newTs.write(properties=False)

        self.outputSetOfTiltSeries.update(newTs)
        self.outputSetOfTiltSeries.write()

        self._store()
      
    def create_parameters_file(self, sampling):
        fn_params = self._getExtraPath(PARAMS_FN)
        f = open(fn_params, 'w')
        pixel_size = sampling * 1e-10
        f.write('nGPUs=1')
        f.write('\nnCpuCores=1')
        f.write('\nPIXEL_SIZE=' + str(pixel_size))
        f.write('\nbeadDiameter=' + str(self.beadDiameter))
        f.write('\nautoAli_max_resolution=' + str(float(self.autoAli_max_resolution)))
        f.write('\nautoAli_min_sampling_rate=' + str(float(self.autoAli_min_sampling_rate)))
        f.write('\nautoAli_max_sampling_rate=' + str(float(self.autoAli_max_sampling_rate)))
        f.write('\nautoAli_iterations_per_bin=' + str(self.autoAli_iterations_per_bin))
        f.write('\nautoAli_n_iters_no_rotation=' + str(self.autoAli_n_iters_no_rotation))
        f.write('\nautoAli_patch_size_factor=' + str(self.autoAli_patch_size_factor))
        f.write('\nautoAli_patch_tracking_border=' + str(self.autoAli_patch_tracking_border))
        f.write('\nautoAli_patch_overlap=' + str(self.autoAli_patch_overlap))
        f.write('\nautoAli_max_shift_in_angstroms=' + str(self.autoAli_max_shift_in_angstroms))
        f.write('\nautoAli_max_shift_factor=' + str(self.autoAli_max_shift_factor))
        f.write('\nautoAli_refine_on_beads=' + str(self.autoAli_refine_on_beads).lower())
        f.close()
        return fn_params
    # --------------------------- INFO functions -----------------------------------
    def _summary(self):
        """ Summarize what the protocol has done"""
        summary = []

        if self.isFinished():
            summary.append("This protocol has printed *%s* %i times." % (self.message, self.times))
        return summary

    def _methods(self):
        methods = []

        if self.isFinished():
            methods.append("%s has been printed in this run %i times." % (self.message, self.times))
            if self.previousCount.hasPointer():
                methods.append("Accumulated count from previous runs were %i."
                               " In total, %s messages has been printed."
                               % (self.previousCount, self.count))
        return methods
