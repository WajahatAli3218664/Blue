import React from 'react';
import styles from './Alien.module.css';

const Alien = () => {
  return (
    <div className={styles.alienContainer}>
      <svg viewBox="0 0 200 300" className={styles.alien}>
        {/* Head */}
        <ellipse cx="100" cy="80" rx="60" ry="70" fill="var(--ifm-color-primary)" opacity="0.9"/>
        <ellipse cx="100" cy="80" rx="55" ry="65" fill="#000000"/>
        
        {/* Eyes */}
        <ellipse cx="75" cy="70" rx="20" ry="30" fill="var(--ifm-color-primary)"/>
        <ellipse cx="125" cy="70" rx="20" ry="30" fill="var(--ifm-color-primary)"/>
        <ellipse cx="75" cy="75" rx="10" ry="15" fill="#000000"/>
        <ellipse cx="125" cy="75" rx="10" ry="15" fill="#000000"/>
        <circle cx="75" cy="70" r="5" fill="var(--ifm-color-primary)"/>
        <circle cx="125" cy="70" r="5" fill="var(--ifm-color-primary)"/>
        
        {/* Mouth */}
        <path d="M 85 100 Q 100 105 115 100" stroke="var(--accent-yellow)" strokeWidth="2" fill="none"/>
        
        {/* Body */}
        <ellipse cx="100" cy="170" rx="40" ry="50" fill="var(--ifm-color-primary)" opacity="0.9"/>
        <ellipse cx="100" cy="170" rx="35" ry="45" fill="#000000"/>
        
        {/* Neck */}
        <rect x="90" y="130" width="20" height="20" fill="var(--ifm-color-primary)" opacity="0.9"/>
        <rect x="92" y="132" width="16" height="16" fill="#000000"/>
        
        {/* Left Arm */}
        <g className={styles.leftArm}>
          <line x1="65" y1="160" x2="30" y2="180" stroke="var(--ifm-color-primary)" strokeWidth="8" strokeLinecap="round"/>
          <line x1="30" y1="180" x2="20" y2="200" stroke="var(--ifm-color-primary)" strokeWidth="8" strokeLinecap="round"/>
          {/* Left Hand */}
          <circle cx="20" cy="200" r="8" fill="var(--ifm-color-primary)"/>
          <line x1="15" y1="205" x2="10" y2="215" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
          <line x1="20" y1="208" x2="18" y2="218" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
          <line x1="25" y1="205" x2="26" y2="215" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
        </g>
        
        {/* Right Arm */}
        <g className={styles.rightArm}>
          <line x1="135" y1="160" x2="170" y2="180" stroke="var(--ifm-color-primary)" strokeWidth="8" strokeLinecap="round"/>
          <line x1="170" y1="180" x2="180" y2="200" stroke="var(--ifm-color-primary)" strokeWidth="8" strokeLinecap="round"/>
          {/* Right Hand */}
          <circle cx="180" cy="200" r="8" fill="var(--ifm-color-primary)"/>
          <line x1="175" y1="205" x2="170" y2="215" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
          <line x1="180" y1="208" x2="178" y2="218" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
          <line x1="185" y1="205" x2="186" y2="215" stroke="var(--ifm-color-primary)" strokeWidth="2"/>
        </g>
        
        {/* Left Leg */}
        <g className={styles.leftLeg}>
          <line x1="85" y1="215" x2="75" y2="260" stroke="var(--ifm-color-primary)" strokeWidth="10" strokeLinecap="round"/>
          {/* Left Foot */}
          <ellipse cx="75" cy="270" rx="15" ry="8" fill="var(--ifm-color-primary)"/>
        </g>
        
        {/* Right Leg */}
        <g className={styles.rightLeg}>
          <line x1="115" y1="215" x2="125" y2="260" stroke="var(--ifm-color-primary)" strokeWidth="10" strokeLinecap="round"/>
          {/* Right Foot */}
          <ellipse cx="125" cy="270" rx="15" ry="8" fill="var(--ifm-color-primary)"/>
        </g>
        
        {/* Glow effect */}
        <ellipse cx="100" cy="80" rx="60" ry="70" fill="none" stroke="var(--accent-yellow)" strokeWidth="2" opacity="0.5"/>
      </svg>
    </div>
  );
};

export default Alien;
