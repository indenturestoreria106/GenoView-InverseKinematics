import struct
import numpy as np
import bvh
import quat
from scipy.ndimage import median_filter, gaussian_filter1d

bvh_files = [
    # 'ground1_subject1.bvh',
    # 'ground2_subject2.bvh',
    # 'kthstreet_gPO_sFM_cAll_d02_mPO_ch01_atombounce_001.bvh',
    'run2_subject4.bvh',
    'walk2_subject4.bvh',
]

joints = [
    'Hips',
    'Spine',
    'Spine1',
    'Spine2',
    'Spine3',
    'Neck',
    'Neck1',
    'Head',
    'HeadEnd',
    'RightShoulder',
    'RightArm',
    'RightForeArm',
    'RightHand',
    'RightHandThumb1',
    'RightHandThumb2',
    'RightHandThumb3',
    'RightHandThumb4',
    'RightHandIndex1',
    'RightHandIndex2',
    'RightHandIndex3',
    'RightHandIndex4',
    'RightHandMiddle1',
    'RightHandMiddle2',
    'RightHandMiddle3',
    'RightHandMiddle4',
    'RightHandRing1',
    'RightHandRing2',
    'RightHandRing3',
    'RightHandRing4',
    'RightHandPinky1',
    'RightHandPinky2',
    'RightHandPinky3',
    'RightHandPinky4',
    'RightForeArmEnd',
    'RightArmEnd',
    'LeftShoulder',
    'LeftArm',
    'LeftForeArm',
    'LeftHand',
    'LeftHandThumb1',
    'LeftHandThumb2',
    'LeftHandThumb3',
    'LeftHandThumb4',
    'LeftHandIndex1',
    'LeftHandIndex2',
    'LeftHandIndex3',
    'LeftHandIndex4',
    'LeftHandMiddle1',
    'LeftHandMiddle2',
    'LeftHandMiddle3',
    'LeftHandMiddle4',
    'LeftHandRing1',
    'LeftHandRing2',
    'LeftHandRing3',
    'LeftHandRing4',
    'LeftHandPinky1',
    'LeftHandPinky2',
    'LeftHandPinky3',
    'LeftHandPinky4',
    'LeftForeArmEnd',
    'LeftArmEnd',
    'RightUpLeg',
    'RightLeg',
    'RightFoot',
    'RightToeBase',
    'RightToeBaseEnd',
    'RightLegEnd',
    'RightUpLegEnd',
    'LeftUpLeg',
    'LeftLeg',
    'LeftFoot',
    'LeftToeBase',
    'LeftToeBaseEnd',
    'LeftLegEnd',
    'LeftUpLegEnd',
]

for bvh_file in bvh_files:
    
    bvh_data = bvh.load(bvh_file)
    
    positions = bvh_data['positions'].copy()
    parents = bvh_data['parents'].copy()
    names = bvh_data['names'].copy()
    rotations = quat.unroll(quat.from_euler(np.radians(bvh_data['rotations']), order=bvh_data['order']))
    rotations, positions = quat.fk(rotations, positions, parents)
    
    assert names == joints
    
    # Convert from cm to m
    positions = (0.01 * positions).astype(np.float32)
    

    # Compute Contacts
    
    velocities = np.sqrt(np.sum(np.square((positions[1:] - positions[:-1]) * 60.0), axis=-1))
    heights = positions[:,:,1]
    
    toes = np.array([ names.index('LeftToeBase'), names.index('RightToeBase') ])
    
    velocity_threshold = 0.25
    # velocity_threshold = 0.2

    left_contact_raw = velocities[:,toes[0]] < velocity_threshold
    right_contact_raw = velocities[:,toes[1]] < velocity_threshold
    left_contact = median_filter(left_contact_raw, size=(5), mode='nearest', axes=0)
    right_contact = median_filter(right_contact_raw, size=(5), mode='nearest', axes=0)
    left_contact_smooth = gaussian_filter1d(left_contact.astype(np.float32), sigma=1.5, mode='nearest', axis=0)
    right_contact_smooth = gaussian_filter1d(right_contact.astype(np.float32), sigma=1.5, mode='nearest', axis=0)
    
    if False:
        
        #####

        import matplotlib.pyplot as plt
        plt.style.use('ggplot')
        
        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        
        # start, end = 600, 800
        start, end = 1050, 1250
        
        #####
        
        fig, ax = plt.subplots(1, 1, sharex=True, figsize=(6.4,3.2), dpi=75)
        
        ax.plot(velocities[start:end,toes[0]], label='left toe')
        ax.plot(velocities[start:end,toes[1]], label='right toe')
        ax.set_xlabel('frame')
        ax.set_ylabel('speed (m/s)')
        ax.legend()
        
        plt.tight_layout()
        plt.show()
        
        #####
        
        fig, ax = plt.subplots(3, 1, sharex=True, height_ratios=[2, 1, 1], figsize=(6.4,6.4), dpi=75)
        
        ax[0].plot(velocities[start:end,toes[0]], label='left toe')
        ax[0].plot(velocities[start:end,toes[1]], label='right toe')
        ax[0].set_xlabel('frame')
        ax[0].set_ylabel('speed (m/s)')
        ax[0].axhline(velocity_threshold, color='grey', linestyle=':', label='threshold')
        ax[0].legend()
        
        ax[1].plot(left_contact_raw[start:end], color=colors[0], label='left toe')
        ax[1].set_xlabel('frame')
        ax[1].set_ylabel('contact')
        ax[1].legend()
        
        ax[2].plot(right_contact_raw[start:end], color=colors[1], label='right toe')
        ax[2].set_xlabel('frame')
        ax[2].set_ylabel('contact')
        ax[2].legend()

        plt.tight_layout()
        plt.show()
        
        #####
        
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(6.4,6.4), dpi=75)
        
        ax[0].plot(velocities[start:end,toes[0]], label='left toe')
        ax[0].plot(velocities[start:end,toes[1]], label='right toe')
        ax[0].set_xlabel('frame')
        ax[0].set_ylabel('speed (m/s)')
        ax[0].legend()
        
        ax[1].plot(heights[start:end,toes[0]], label='left toe')
        ax[1].plot(heights[start:end,toes[1]], label='right toe')
        ax[1].set_xlabel('frame')
        ax[1].set_ylabel('height (m)')
        ax[1].legend()

        plt.tight_layout()
        plt.show()
        
        #####
        
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(6.4,6.4), dpi=75)
        
        ax[0].plot(left_contact_raw[start:end], color=colors[0], label='left toe')
        ax[0].set_xlabel('frame')
        ax[0].set_ylabel('contact')
        ax[0].legend()
        
        ax[1].plot(right_contact_raw[start:end], color=colors[1], label='right toe')
        ax[1].set_xlabel('frame')
        ax[1].set_ylabel('contact')
        ax[1].legend()
        
        ax[2].plot(left_contact[start:end], color=colors[0], label='left toe (filtered)')
        ax[2].set_xlabel('frame')
        ax[2].set_ylabel('contact')
        ax[2].legend()
        
        ax[3].plot(right_contact[start:end], color=colors[1], label='right toe (filtered)')
        ax[3].set_xlabel('frame')
        ax[3].set_ylabel('contact')
        ax[3].legend()

        plt.tight_layout()
        plt.show()
        
        #####
        
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(6.4,6.4), dpi=75)
        
        ax[0].plot(left_contact[start:end], color=colors[0], label='left toe (filtered)')
        ax[0].set_xlabel('frame')
        ax[0].set_ylabel('contact')
        ax[0].legend()
        
        ax[1].plot(right_contact[start:end], color=colors[1], label='right toe (filtered)')
        ax[1].set_xlabel('frame')
        ax[1].set_ylabel('contact')
        ax[1].legend()
        
        ax[2].plot(left_contact_smooth[start:end], color=colors[0], label='left toe (smoothed)')
        ax[2].set_xlabel('frame')
        ax[2].set_ylabel('contact')
        ax[2].legend(loc='upper right')
        
        ax[3].plot(right_contact_smooth[start:end], color=colors[1], label='right toe (smoothed)')
        ax[3].set_xlabel('frame')
        ax[3].set_ylabel('contact')
        ax[3].legend(loc='upper right')

        plt.tight_layout()
        plt.show()
        
        #####
    
    # Introduce manual sliding once contacts have been computed
    
    # root_scale_factor = 1.5
    root_scale_factor = 0.75
    
    rotations, positions = quat.ik(rotations, positions, parents)
    positions[:,0] *= np.array([root_scale_factor, 1.0, root_scale_factor])
    
    rotations, positions = quat.fk(rotations, positions, parents)
    
    with open(bvh_file.replace('.bvh', '.bin'), 'wb') as f:
        
        nframes = positions.shape[0]
        nbones = positions.shape[1]

        f.write(struct.pack('ii', nframes, nbones))
        
        for i in range(nbones):
            f.write(struct.pack('32si', bytes(names[i], encoding='ascii'), parents[i]))
        
        for i in range(nframes):
            for j in range(nbones):
                f.write(struct.pack('ffffffffff',
                    positions[i,j,0], positions[i,j,1], positions[i,j,2],
                    rotations[i,j,1], rotations[i,j,2], rotations[i,j,3], rotations[i,j,0],
                    1.0, 1.0, 1.0
                ))
            
        f.write(struct.pack('32s', bytes(bvh_file.replace('.bvh',''), encoding='ascii')[:31]))
        
    
    with open(bvh_file.replace('.bvh', '_contacts.bin'), 'wb') as f:
        f.write(struct.pack('i', positions.shape[0]))
        f.write(left_contact_smooth.tobytes())
        f.write(right_contact_smooth.tobytes())

    
    
    
    
    
    