/**
 * Analytics service for tracking user onboarding completion rates
 */

export interface AnalyticsEvent {
  event: string;
  properties?: Record<string, any>;
  timestamp: Date;
}

class AnalyticsService {
  private endpoint: string;
  private userId: string | null = null;
  private sessionId: string | null = null;

  constructor(endpoint: string = '/api/analytics') {
    this.endpoint = endpoint;
    this.generateSessionId();
  }

  setUserId(userId: string) {
    this.userId = userId;
  }

  generateSessionId() {
    this.sessionId = 'sess_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  async track(event: string, properties?: Record<string, any>) {
    try {
      const analyticsEvent: AnalyticsEvent = {
        event,
        properties: {
          ...properties,
          userId: this.userId,
          sessionId: this.sessionId,
          timestamp: new Date(),
        },
        timestamp: new Date(),
      };

      // In a real implementation, this would send to an analytics backend
      console.log('Analytics event:', analyticsEvent);

      // For development, we'll just log the event
      // In production, uncomment the fetch call below
      /*
      await fetch(this.endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(analyticsEvent),
      });
      */

      return true;
    } catch (error) {
      console.error('Failed to track analytics event:', error);
      return false;
    }
  }

  // Track user engagement
  async trackUserEngagement(properties?: Record<string, any>) {
    return this.track('user_engagement', {
      ...properties,
      step: 'engagement',
    });
  }

  // Track content interaction
  async trackContentInteraction(contentId: string, properties?: Record<string, any>) {
    return this.track('content_interaction', {
      ...properties,
      contentId,
    });
  }

  // Track documentation progress
  async trackDocumentationProgress(properties?: Record<string, any>) {
    return this.track('documentation_progress', properties);
  }

  // Calculate onboarding completion rate
  async getOnboardingCompletionRate(): Promise<number> {
    // In a real implementation, this would query analytics data
    // For now, we'll return a mock value
    return 0.75; // 75% completion rate
  }
}

// Create a singleton instance
export const analyticsService = new AnalyticsService();

// Export for use in components
export default analyticsService;