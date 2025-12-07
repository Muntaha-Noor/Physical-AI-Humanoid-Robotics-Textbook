<!--
---
Sync Impact Report
---
- **Version Change**: `0.1.0` -> `1.0.0`
- **Principles Added**:
  - I. Technical Accuracy and Traceability
  - II. Educational Clarity and Accessibility
  - III. Reproducibility and Testability
  - IV. Content and Structural Integrity
  - V. RAG Chatbot Integrity
- **Templates Requiring Updates**:
  - [ ] `.specify/templates/plan-template.md`
  - [ ] `.specify/templates/spec-template.md`
  - [ ] `.specify/templates/tasks-template.md`
- **Follow-up TODOs**: None
-->
# Physical AI & Humanoid Robotics — A complete Docusaurus-based textbook with an integrated RAG chatbot. Constitution

## Core Principles

### I. Technical Accuracy and Traceability
All factual claims, technical explanations, and code examples must be traceable to official documentation, primary robotics sources, or peer-reviewed research. All claims must be verifiable, and code must align with the current stable versions of the respective frameworks (ROS 2, Gazebo, Isaac, VLA). A strict zero-plagiarism policy is in effect.
***Rationale**: To establish the textbook as a credible, authoritative educational resource for advanced students and practitioners in robotics and AI.*

### II. Educational Clarity and Accessibility
The writing style must be clear, concise, and targeted at an advanced undergraduate or graduate-level audience (approximating a grade 9–12 readability level). All diagrams, examples, and code must follow a unified style to ensure a cohesive learning experience.
***Rationale**: To make complex robotics concepts understandable and to provide a consistent, high-quality educational experience for the reader.*

### III. Reproducibility and Testability
All code, tutorials, and data pipelines (including the RAG chatbot) must be fully reproducible from the provided instructions and artifacts. All code must be tested on the specified versions of the frameworks. The final capstone project must be completable by a student following the textbook.
***Rationale**: To ensure that students can practically apply the concepts learned and to validate the correctness of the provided materials.*

### IV. Content and Structural Integrity
The project must meet the minimum content requirements: 12+ chapters, 25,000+ words, and 30+ credible sources. The Docusaurus site must build and deploy successfully to GitHub Pages. The RAG chatbot must be fully functional and integrated within the deployed site.
***Rationale**: To ensure the final product is a substantial, complete, and functional educational resource that meets its stated goals.*

### V. RAG Chatbot Integrity
The RAG chatbot must strictly derive its answers from the book's content. The entire pipeline, from ingestion and embedding to the Qdrant database and FastAPI backend, must be fully documented, configured, and reproducible.
***Rationale**: To provide a reliable and accurate study companion that reinforces the textbook's content without introducing external, unverified information.*

## Governance

This Constitution is the single source of truth for project principles and standards. All development, contributions, and reviews MUST adhere to it. Amendments require a formal proposal, review, and an approved migration plan to update all affected project artifacts.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07