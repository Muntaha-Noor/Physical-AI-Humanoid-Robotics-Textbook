import React, { createContext, useContext, useEffect, useState } from 'react';

// Define the shape of our language context
interface LanguageContextType {
  language: 'en' | 'ur';
  toggleLanguage: () => void;
  t: (key: string) => string; // Translation function
}

// Create the context with default values
const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

// Custom hook to use the language context
export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};

// Translation mapping
const translations = {
  'Course Overview': {
    en: 'Course Overview',
    ur: 'کورس کا جائزہ'
  },
  'Course Fundamentals': {
    en: 'Course Fundamentals',
    ur: 'کورس کے بنیادی عناصر'
  },
  'Course Introduction: Foundations of Physical AI': {
    en: 'Course Introduction: Foundations of Physical AI',
    ur: 'کورس کا تعارف: جسمانی مصنوعی ذہانت کی بنیادیں'
  },
  'Module 1: ROS 2 Basics': {
    en: 'Module 1: ROS 2 Basics',
    ur: 'مODULE 1: ROS 2 کی بنیادیں'
  },
  'Module 2: Simulation Basics – Gazebo': {
    en: 'Module 2: Simulation Basics – Gazebo',
    ur: 'مODULE 2: سیمیولیشن کی بنیاد – گزیبو'
  },
  'Module 2: Simulation Basics – Unity': {
    en: 'Module 2: Simulation Basics – Unity',
    ur: 'مODULE 2: سیمیولیشن کی بنیاد – یونٹی'
  },
  'Key Concepts in Physical AI': {
    en: 'Key Concepts in Physical AI',
    ur: 'جسمانی مصنوعی ذہانت کے اہم تصورات'
  },
  'Advanced ROS 2 Topics': {
    en: 'Advanced ROS 2 Topics',
    ur: 'ROS 2 کے اعلیٰ موضوعات'
  },
  'Advanced Topics': {
    en: 'Advanced Topics',
    ur: 'اعلیٰ موضوعات'
  },
  'Module 3: NVIDIA Isaac AI and Perception': {
    en: 'Module 3: NVIDIA Isaac AI and Perception',
    ur: 'مODULE 3: NVIDIA Isaac AI اور ادراک'
  },
  'Module 4: Vision-Language-Action (VLA)': {
    en: 'Module 4: Vision-Language-Action (VLA)',
    ur: 'مODULE 4: وژن-زبان-ایکشن (VLA)'
  },
  'Physical AI & Humanoid Robotics': {
    en: 'Physical AI & Humanoid Robotics',
    ur: 'جسمانی مصنوعی ذہانت اور ہیومنوائڈ روبوٹکس'
  },
  'Explore the Textbook': {
    en: 'Explore the Textbook',
    ur: ' textbook کا جائزہ لیں'
  },
  'A comprehensive guide to cutting-edge Physical AI and humanoid robotics with interactive RAG chatbot.': {
    en: 'A comprehensive guide to cutting-edge Physical AI and humanoid robotics with interactive RAG chatbot.',
    ur: 'جسمانی مصنوعی ذہانت اور ہیومنوائڈ روبوٹکس کے بارے میں جامع ہدایات، میں تعاملی RAG چیٹ بٹ کے ساتھ.'
  },
  'What You\'ll Learn': {
    en: 'What You\'ll Learn',
    ur: 'آپ کیا سیکھیں گے'
  },
  'Module 1: The Robotic Nervous System (ROS 2)': {
    en: 'Module 1: The Robotic Nervous System (ROS 2)',
    ur: 'مODULE 1: روبوٹک اعصابی نظام (ROS 2)'
  },
  'Module 2: The Digital Twin (Gazebo & Unity)': {
    en: 'Module 2: The Digital Twin (Gazebo & Unity)',
    ur: 'مODULE 2: ڈیجیٹل ٹوئن (Gazebo & Unity)'
  },
  'Module 3: The AI-Robot Brain (NVIDIA Isaac)': {
    en: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    ur: 'مODULE 3: AI-روبوٹ دماغ (NVIDIA Isaac)'
  },
  'Module 4: Vision-Language-Action (VLA)': {
    en: 'Module 4: Vision-Language-Action (VLA)',
    ur: 'مODULE 4: وژن-زبان-ایکشن (VLA)'
  },
  'Capstone Project': {
    en: 'Capstone Project',
    ur: 'کیپ اسٹون پروجیکٹ'
  },
  'A hands-on project where you will integrate all learned concepts to build an autonomous humanoid robot.': {
    en: 'A hands-on project where you will integrate all learned concepts to build an autonomous humanoid robot.',
    ur: 'ایک عملی پروجیکٹ جہاں آپ سب سیکھے ہوئے تصورات کو ضم کریں گے تاکہ ایک خودمختار ہیومنوائڈ روبوٹ تیار کیا جا سکے.'
  }
};

// Language provider component
export const LanguageProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');

  // Load language preference from localStorage on initial render
  useEffect(() => {
    const savedLanguage = localStorage.getItem('language') as 'en' | 'ur' | null;
    if (savedLanguage) {
      setLanguage(savedLanguage);
    } else {
      // Default to English
      setLanguage('en');
    }
  }, []);

  // Apply language to the document and save to localStorage
  useEffect(() => {
    if (typeof document !== 'undefined') {
      // Update the lang attribute
      document.documentElement.setAttribute('lang', language);
      // Save to localStorage
      localStorage.setItem('language', language);
    }
  }, [language]);

  const toggleLanguage = () => {
    setLanguage(prevLang => {
      const newLang = prevLang === 'en' ? 'ur' : 'en';
      return newLang;
    });
  };

  const t = (key: string): string => {
    // Return the translation if available, otherwise return the original key
    return translations[key] ? translations[key][language] : key;
  };

  return (
    <LanguageContext.Provider value={{ language, toggleLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
};

export default LanguageContext;