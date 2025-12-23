import React, { useEffect } from 'react';

const Speckit = () => {
  useEffect(() => {
    // Spec-Kit Plus Integration
    // This component demonstrates the use of Spec-Kit Plus for AI-driven development
    if (typeof window !== 'undefined') {
      // Log Spec-Kit Plus usage
      console.log('Built with Spec-Kit Plus: https://github.com/panaversity/spec-kit-plus/');
      console.log('Developed using Claude Code: https://www.claude.com/product/claude-code');
      console.log('For Panaversity Hackathon I: https://panaversity.org');
    }
  }, []);

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      left: '20px',
      background: 'linear-gradient(135deg, var(--ifm-color-primary), var(--accent-yellow))',
      color: '#ffffff',
      padding: '8px 16px',
      borderRadius: '20px',
      fontSize: '11px',
      fontWeight: 'bold',
      boxShadow: '0 6px 20px rgba(59,130,246,0.12)',
      zIndex: 998,
      display: 'flex',
      alignItems: 'center',
      gap: '8px'
    }}>
      <span>🚀</span>
      <a 
        href="https://github.com/panaversity/spec-kit-plus/" 
        target="_blank" 
        rel="noopener noreferrer"
        style={{
          color: 'var(--ifm-color-content)',
          textDecoration: 'none',
          fontWeight: 'bold'
        }}
      >
        Built with Spec-Kit Plus
      </a>
    </div>
  );
};

export default Speckit;
