# VERSIONING i RELEASE PROCESS

Ten dokument opisuje sposób wersjonowania kodu i reguły tworzenia nowych wersji plików przy zachowaniu zasady "nie nadpisujemy istniejących plików".

1. Zasady ogólne
- Projekt używa semantycznego wersjonowania (MAJOR.MINOR.PATCH) dla wydań (releases).
- Dodatkowo, pliki krytyczne (np. szkice .ino, wygenerowane nagłówki) muszą mieć numer wersji w nazwie pliku.

2. Nazewnictwo plików
- Format: <base_name>_v<MAJOR>.<MINOR>.<PATCH><_label>.ext
  - Przykład: master_kc868_a16_multi_sid_v21.2.0.ino
  - Alternatywnie dla drobnych poprawek: master_kc868_a16_multi_sid_v21.2.0_patch1.ino lub _v21.2.0_20251210.ino
- Unikaj nadmiernie długich sufiksów; trzymaj wersjonowanie czytelne.

3. Tworzenie nowej wersji pliku
- Skopiuj oryginalny plik i zmodyfikuj kopię.
- Zmień nazwę według konwencji i zaktualizuj nagłówki komentarzy (np. opis wersji, datę, autora).
- Dodaj wpis do CHANGELOG.md (jeśli istnieje) z krótkim opisem zmian.
- Otwórz PR do main z nowym plikiem.

4. Releases (etykiety GitHub)
- Po zaakceptowaniu PR i scaleniu, stwórz GitHub Release oznaczony semver (np. v21.2.0) i w treści release umieść listę nowych plików oraz linki do PR.

5. Branch protection (rekomendowane ustawienia repo)
- Protect branch: main
  - Require pull request reviews before merging (min 1 reviewer)
  - Require status checks to pass before merging (np. CI build)
  - Require linear history (optional)
  - Include administrators (tak, aby admini również podlegali regułom)
  - Dismiss stale pull request approvals when new commits are pushed
  - Require signed commits (opcjonalnie)

6. CI i kontrola nadpisywania plików
- Dodaj prosty workflow (GitHub Actions) uruchamiany na PR, który:
  - Sprawdza, czy żaden z istniejących plików w repo nie został zmodyfikowany (fail jeśli detect edit). Przydatny snippet: porównać listę zmienionych plików i fail gdy któryś z nich ma nazwę istniejącą w repoząry (można wymagać nowego pliku zamiast zmiany).
  - Uruchamia podstawową kompilację (opcjonalnie) dla ESP32/ESP8266.

7. Emergency fixes
- Jeżeli wymagana jest szybka, krytyczna poprawka, nadal NIE nadpisujemy istniejących plików.
- Procedura: utworzyć nowy plik z prefiksem emergency_ lub suffixem _hotfix oraz otworzyć PR z wysokim priorytetem; w opisie PR zaznaczyć, że to hotfix i uzasadnienie.

8. Przykładowy workflow PR
- Tworzysz branch feature/x
- Dodajesz nową wersję pliku (np. myfile_v1.1.0.ino)
- Aktualizujesz CHANGELOG.md
- Otwierasz PR do main, dołączasz checklistę PR
- Po review i zielonym CI, scalasz PR
- Tworzysz GitHub Release vMAJOR.MINOR.PATCH

---

Dziękuję — gdy potwierdzisz, mogę również przygotować przykładowy GitHub Actions workflow (pliki .github/workflows/verify_no_overwrite.yml i build.yml) oraz szablon PR (PULL_REQUEST_TEMPLATE.md).
