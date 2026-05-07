use anyhow::Result;
use std::collections::HashSet;
use std::fs;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct AgreementDecision {
    pub agreed_altitude_ft: Option<i32>,
    pub handoff_fix: Option<String>,
}

#[derive(Debug, Clone)]
pub struct AgreementResolver {
    rules: Vec<AgreementRule>,
}

#[derive(Debug, Clone)]
struct AgreementRule {
    departure_filter: String,
    arrival_filter: String,
    entry_fix_filter: String,
    agreed_altitude_ft: Option<i32>,
    handoff_fix: Option<String>,
}

impl AgreementResolver {
    pub fn load_from_dir<P: AsRef<Path>>(agreements_dir: P) -> Result<Self> {
        let mut rules = Vec::new();
        let mut stack = vec![agreements_dir.as_ref().to_path_buf()];

        while let Some(dir) = stack.pop() {
            if !dir.exists() || !dir.is_dir() {
                continue;
            }

            for entry in fs::read_dir(&dir)? {
                let entry = entry?;
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                    continue;
                }

                let is_txt = path
                    .extension()
                    .and_then(|ext| ext.to_str())
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("txt"));
                if !is_txt {
                    continue;
                }

                rules.extend(Self::parse_rules_file(&path)?);
            }
        }

        Ok(Self { rules })
    }

    pub fn resolve_internal_transit(
        &self,
        departure: &str,
        arrival: &str,
        route_fixes: &[String],
    ) -> Option<AgreementDecision> {
        let departure = departure.trim().to_uppercase();
        let arrival = arrival.trim().to_uppercase();
        let route_set: HashSet<String> = route_fixes
            .iter()
            .map(|fix| fix.trim().to_uppercase())
            .filter(|fix| !fix.is_empty())
            .collect();

        let mut best: Option<(&AgreementRule, i32)> = None;
        for rule in &self.rules {
            let Some(score) = rule.match_score(&departure, &arrival, &route_set) else {
                continue;
            };

            if best.as_ref().is_none_or(|(_, best_score)| score > *best_score) {
                best = Some((rule, score));
            }
        }

        best.map(|(rule, _)| AgreementDecision {
            agreed_altitude_ft: rule.agreed_altitude_ft,
            handoff_fix: rule.handoff_fix.clone(),
        })
    }

    fn parse_rules_file(path: &Path) -> Result<Vec<AgreementRule>> {
        let content = fs::read_to_string(path)?;
        let mut rules = Vec::new();

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with(';') {
                continue;
            }

            let parts: Vec<&str> = trimmed.split(':').collect();
            if parts.len() < 11 || !parts[0].eq_ignore_ascii_case("COPX") {
                continue;
            }

            let departure_filter = parts[1].trim().to_uppercase();
            let entry_fix_filter = parts[3].trim().to_uppercase();
            let arrival_filter = parts[4].trim().to_uppercase();
            let agreed_altitude_ft = parts[8..=10]
                .iter()
                .find_map(|value| parse_optional_altitude(value));

            let handoff_raw = parts[10].trim().to_uppercase();
            let handoff_fix = parse_fix_token(&handoff_raw).or_else(|| parse_fix_token(&entry_fix_filter));

            rules.push(AgreementRule {
                departure_filter,
                arrival_filter,
                entry_fix_filter,
                agreed_altitude_ft,
                handoff_fix,
            });
        }

        Ok(rules)
    }
}

impl AgreementRule {
    fn match_score(
        &self,
        departure: &str,
        arrival: &str,
        route_fixes: &HashSet<String>,
    ) -> Option<i32> {
        let departure_matches = matches_filter(&self.departure_filter, departure)
            || route_fixes.contains(&self.departure_filter);
        if !departure_matches {
            return None;
        }

        let arrival_matches = matches_filter(&self.arrival_filter, arrival)
            || route_fixes.contains(&self.arrival_filter);
        if !arrival_matches {
            return None;
        }

        if !is_wildcard(&self.entry_fix_filter) && !route_fixes.contains(&self.entry_fix_filter) {
            return None;
        }

        let mut score = 0;
        if self.departure_filter == departure {
            score += 4;
        } else if route_fixes.contains(&self.departure_filter) {
            score += 2;
        }
        if self.arrival_filter == arrival {
            score += 4;
        } else if route_fixes.contains(&self.arrival_filter) {
            score += 2;
        }
        if !is_wildcard(&self.entry_fix_filter) {
            score += 2;
        }
        if self.agreed_altitude_ft.is_some() {
            score += 1;
        }
        if self.handoff_fix.is_some() {
            score += 1;
        }

        Some(score)
    }
}

fn is_wildcard(value: &str) -> bool {
    value.trim().is_empty() || value.trim() == "*"
}

fn matches_filter(filter: &str, value: &str) -> bool {
    is_wildcard(filter) || filter.eq_ignore_ascii_case(value)
}

fn parse_optional_altitude(raw: &str) -> Option<i32> {
    let text = raw.trim();
    if text.is_empty() || text == "*" {
        return None;
    }

    text.parse::<i32>().ok()
}

fn parse_fix_token(raw: &str) -> Option<String> {
    let text = raw.trim().to_uppercase();
    if text.is_empty() || text == "*" || text.starts_with('^') {
        return None;
    }

    let mut token = String::new();
    for ch in text.chars() {
        if ch.is_ascii_alphabetic() {
            token.push(ch);
        } else if !token.is_empty() {
            break;
        }
    }

    if token.len() >= 2 {
        Some(token)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_fix_token() {
        assert_eq!(parse_fix_token("SABER5").as_deref(), Some("SABER"));
        assert_eq!(parse_fix_token("LAM15").as_deref(), Some("LAM"));
        assert_eq!(parse_fix_token("^HDG"), None);
    }
}
